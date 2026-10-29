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

Pi は `cat ≥ 0.35` かつ `snoo ≥ 0.6` の中から `cat × snoo` が最大の箱を snoo とみなす（`config.toml` の `[control]` で変えられる）。

## 学習用の画像

Google フォトの共有アルバム（snoo の写真 300 枚）を home-linux の
`~/orca/rocm_opencv_server/data/snoo_raw/img_0001.jpg`〜`img_0300.jpg`（長辺 1280 px）に置いた。
全部に snoo が写っている前提だが、確かめていない（他の猫・人・物が混じっているかもしれない）。

## 試す

Pi 側の偽サーバー: `python -m snoo_trucar fake-server`（画像を受けて、決まった位置に snoo がいると返す）。
Pi から 1 枚送って返事を見る: `python -m snoo_trucar probe --image cat.jpg`。
