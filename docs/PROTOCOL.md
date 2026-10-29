# 検出サーバーとのやりとり（85pi4 ⇄ home-linux）

85pi4（PiCar-X）がカメラの画像を home-linux（Radeon RX 7900 XTX）に送り、home-linux が猫を検出して
snoo かどうかを判定し、結果を返す。遅れを小さくするため **UDP** で、1 フレーム = 1 データグラム。

- 宛先: 家の中では `home-linux.local:8710/udp`（LAN。tailscale 経由より往復が約 3 ms 短い）。名前が引けない・返事が 1 秒来ないときは `home-linux.taila713c8.ts.net:8710/udp`（tailscale）に替え、30 秒ごとに LAN を試し直す
- サーバーは 0.0.0.0 で受け、認証の無い UDP なので、決めた送り元（85pi4 の LAN と tailscale の IP）以外は捨てる
- 返事は、送ってきたアドレス・ポートにそのまま返す
- 目標: 撮ってから返事が届くまで **50 ms 以下**（推論 15 ms 以下）、60 fps を捌けること（今は往復 16〜19 ms、サーバーで 8 ms）
- 届かない・遅れたフレームは捨ててよい（Pi は返事を待たずに最大 2 フレームまで送り、500 ms より古い返事は捨てる）

## Pi → サーバー（画像）

リトルエンディアンの 20 バイトのヘッダーの後に JPEG。

| オフセット | 型 | 内容 |
| --- | --- | --- |
| 0 | 4 bytes | `SNO1` |
| 4 | uint32 | 連番（seq） |
| 8 | uint64 | 撮った時刻（Pi の `CLOCK_MONOTONIC`、ns。画面の中央の行の露光の真ん中）。サーバーは意味を解釈せず、そのまま返す |
| 16 | uint16 | 幅（px、既定 640） |
| 18 | uint16 | 高さ（px、既定 480） |
| 20 | … | JPEG（品質 70、だいたい 20〜40 KB。最大 65000 バイト） |

Python では `struct.Struct("<4sIQHH")`（`snoo_trucar/protocol.py`）。

### 版 2（SNO2、既定）

撮った時刻に書いていた指令を足した 37 バイトのヘッダー。サーバーは SNO1 も SNO2 も読む（2026-10-29 19:52 から）。
`config.toml` の `[server] header_v2 = false` で SNO1 に戻せる。

| オフセット | 型 | 内容 |
| --- | --- | --- |
| 0 | 4 bytes | `SNO2` |
| 4〜19 | | SNO1 と同じ（seq・撮った時刻・幅・高さ） |
| 20 | float32 | ハンドル（度、右が +） |
| 24 | float32 | 速さ（picarx の -100..100、前が +） |
| 28 | float32 | パン（度、右が +） |
| 32 | float32 | チルト（度、上が +。較正後は 0 が水平） |
| 36 | uint8 | flags。bit0 = 走っている、bit1 = 下がっている |
| 37 | … | JPEG |

`struct.Struct("<4sIQHHffffB")`。値は指令で、サーボやモーターの実際の値ではない。

## サーバー → Pi（検出結果）

UTF-8 の JSON 1 つ。

```json
{
  "v": 1,
  "seq": 1234,
  "t_capture_ns": 123456789012,
  "infer_ms": 8.2,
  "dets": [
    {"box": [0.41, 0.30, 0.62, 0.71], "cat": 0.91, "snoo": 0.87}
  ]
}
```

| 項目 | 内容 |
| --- | --- |
| `v` | 版。今は 1 |
| `seq`・`t_capture_ns` | 受け取った画像のヘッダーの値をそのまま |
| `infer_ms` | サーバーで受け取ってから返すまで（ms） |
| `dets` | 猫の検出。見つからなければ空の配列（**空でも必ず返す**。Pi は返事の有無で通信の状態を見る） |
| `dets[].box` | `[x1, y1, x2, y2]`。画像の幅・高さに対する比（0..1）。左上が原点 |
| `dets[].cat` | 猫である確からしさ（0..1） |
| `dets[].snoo` | snoo である確からしさ（0..1）。判定できないときは `null`（Pi 側は `require_snoo = false` で猫なら追う） |
| `dets[].play` | 任意。遊びの様子 `{"state": "stalk", "p": {"attention", "stalk", "pounce", "bored", "stress"}}`（各 0..1）。判定できないときは `null` か省く。Pi は今 `p.stress` だけを使う（`snoo_trucar/guard.py`: 0.8 以上を 1 回、0.5 以上を 3 回続けて見たら止まって下がる） |
| `clear_cm`・`clear_conf` | 任意（上の階層）。走っている間の、これから走る弧の通り道で最初の障害物までの距離（cm）と確からしさ。**欄が無い** = 出していない（Pi は超音波だけで走る）。**null** = 出そうとしたが間に合わなかった（Pi は null が 0.3 s 続いたら止まって撮り直す）。値が `explore_clear_stop_cm`（既定 25）より近ければ、進むのをやめて向きを変える。大きく曲がっていて通り道が画面に入らないとき・下がっているときは欄を出さず、`clear_skip`（`"out_of_view"` / `"reversing"`）を出す |

Pi は `cat ≥ 0.35` かつ `snoo ≥ 0.6` の中から `cat × snoo` が最大の箱を snoo とみなす（`config.toml` の `[control]` で変えられる）。

## 学習用の画像

Google フォトの共有アルバム（snoo の写真 300 枚）を home-linux の
`~/orca/rocm_opencv_server/data/snoo_raw/img_0001.jpg`〜`img_0300.jpg`（長辺 1280 px）に置いた。
全部に snoo が写っている前提だが、確かめていない（他の猫・人・物が混じっているかもしれない）。

## 試す

Pi 側の偽サーバー: `python -m snoo_trucar fake-server`（画像を受けて、決まった位置に snoo がいると返す）。
Pi から 1 枚送って返事を見る: `python -m snoo_trucar probe --image cat.jpg`。

## 探索の地図（station、HTTP）

explore で止まって撮るたびに（station）、`POST http://192.168.0.248:8720/v1/snoo/map/station`（multipart/form-data）で送り、
次の 1 歩をもらう（`snoo_trucar/station.py`。サーバーは home-linux の rocm_opencv_server）。

- `meta`: scan.json の形の JSON。`frames[]` はこの station の画像（前の station から走ってくる間の分も）。
  加えて `session_id`（explore_日時）、`station_id`（0, 1, …）、`prev_step`（`{steer_deg, speed, seconds, est_distance_cm, stop_reason}`、
  最初は null、Pi だけで決めた後は `{"stop_reason": "local"}`）、`battery_v`、`ultrasonic_cm`、`camera_height_cm`（8.0、ユーザーが測った）
- `images`: 1 枚ずつ。ファイル名は `frames[].file`
- 返事: `{"action": "move", "steer_deg", "speed", "distance_cm", "pose": {x, y, yaw_deg}}` か `{"action": "done", "reason"}`。
  meta の `"supports": ["reverse", "segments"]` を見て、サーバーは次も返す: `"reverse": true`（同じハンドルで下がる。30 cm まで）、
  `"segments": [{"steer_deg", "distance_cm"}, …]`（曲がってからまっすぐ、など。3 つまで。`"long_step"` で最後のまっすぐは 60 cm まで、Pi は合わせて 80 cm で切る）。
  `prev_step` には `reverse` と、走った `segments` も入れる。
  Pi は速さを `explore_speed`、1 歩を `explore_step_cm` までに抑え、前に進むときは超音波（`explore_step_stop_cm`、30 cm）・clear_cm、
  どちらの向きでも画像が動かないときは途中で止める
  （`stop_reason`: done / ultrasonic / clear / clear_unknown / stuck）
- 20 s で返事が無い・失敗したら、Pi だけで次を決める（超音波で前が空いていれば進み、塞がっていれば切り返す）
- 地図は `GET http://192.168.0.248:8720/v1/snoo/map/<session_id>/topdown.png`
