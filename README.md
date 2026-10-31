# snoo_trucar

SunFounder の PiCar-X（85pi4、Raspberry Pi 4）で、飼い猫の snoo を探して、カメラで追い、距離を保って追いかけたり離れたりする。
猫の検出と snoo の判定は home-linux（Radeon RX 7900 XTX）の検出サーバーで行う（`docs/PROTOCOL.md`）。

## 使う（USR ボタン）

Robot HAT の USR ボタンで行動を選ぶ。LED の点滅の回数が今の行動。

| 押し方 | 行動 | 内容 |
| --- | --- | --- |
| 1 回 | watch | snoo を探す。カメラを左右に振り、見つけたらカメラで追い続ける。車体は動かない（`search_drive = true` なら探すときに少し向きを変える） |
| 2 回 | keep | 距離を保つ。遠ければ近づき、近すぎれば下がる（`keep_distance_cm`、既定 80 cm） |
| 3 回 | map | 部屋を撮る。その場でカメラを 8 方向 × 2 段に向けて 1 枚ずつ撮り、`~/captures/snoo_map/map_日時/` に画像と `scan.json`（向き・超音波）を置く。一巡したら待機に戻る。地図を組み立てるのは home-linux |
| 4 回 | explore | 部屋を動きながら撮る。止まって首を振って撮る（既定 5 方向 × 2 段。`[control] explore_scan_tilts`・`explore_scan_pans` で変えられる）→ home-linux の地図のサーバーに送り、返ってきた 1 歩（区切りごとに前・後ろ、5 つまで）をそのとおり走る（その間も 5 fps で撮る）、を繰り返す。道はサーバーが決め、Pi は決めない（届かなければ 3 秒おきに送り直して待つ）。前へ走る区切りは clear_cm（カメラ）・guard・画像が動かない・電池・USR で止まる。超音波では止まらない（地図の弱い証拠として送るだけ）。`~/captures/snoo_map/explore_日時/` に画像と `scan.json`（向き・指令・超音波）。snoo が 150 cm より近くに見えたら止まって見守る。10 分で待機に戻る |
| 長押し（1.5 秒） | idle | 止まって待つ（LED は 2 秒に 1 回短く光る） |

RST ボタンは Robot HAT のマイコンのリセットなので使わない。

explore の間は `~/.local/state/snoo_trucar/explore.json` に session_id・次の station・今の段階を書く。
`--mode` を決めずに起動し直したとき、それが 10 分以内なら同じセッションの続きの station から explore に戻る
（まずその場で首を振って撮り、`prev_step` は null。走るのはサーバーの返事が来てから）。explore をやめたら
（長押し・調べ終えた・時間切れ）消す。コードを入れ替えるときは、撮っている間に起動し直す：
`.venv/bin/python -m snoo_trucar wait-scan && systemctl --user restart snoo-trucar`（地図のサーバーの
`/v1/snoo/map/busy` も見る。explore でなければすぐ通る）
コマンドで追いかける行動（follow、50 cm まで近づき、下がらない）も選べる。

```sh
systemctl --user start snoo-trucar          # 待機で起動して、USR ボタンを待つ
journalctl --user -u snoo-trucar -f         # 状態（0.5 秒ごと）

# 手で動かす（サービスを止めてから。カメラを開けるのは 1 プロセスだけ）
.venv/bin/python -m snoo_trucar run --mode watch
.venv/bin/python -m snoo_trucar run --mode keep --max-speed 15
.venv/bin/python -m snoo_trucar run --sim --mode keep    # ハードもサーバーも無しのシミュレーション
.venv/bin/python -m snoo_trucar stop                     # モーターだけ止める（サービスの ExecStopPost もこれ）
```

状態の行の例: `keep TRACK pan=-12.6 … | 方位=-12.6° 距離=45cm … | loop=4993Hz p99遅れ=0.5us | 送信=… rtt=20ms 推論=11ms`

## 仕組み

```
85pi4                                                          home-linux
┌ メインプロセス ─────────────────────────────┐              ┌──────────────────────┐
│ カメラ 640x480 60fps → JPEG ──────── UDP 8710 ───────────→ │ 猫の検出・snoo の判定 │
│ 返事 → snoo の箱を選ぶ → 共有メモリ ←─────────────────────── │（RT-DETR、GPU）       │
│ 超音波 15 Hz → 共有メモリ                    │              └──────────────────────┘
│ USR ボタン・LED → 行動を共有メモリへ          │
│ 50 Hz: 共有メモリのコマンド → I2C（サーボ・モーター）│
└────────────────────────────────────────────┘
        ↑↓ 共有メモリ（seqlock、ロックなし）
┌ 制御プロセス（CPU 3 に固定、1 kHz）─────────┐
│ カルマンフィルタ（方位・仰角・距離、等速）の予測  │
│ 検出が来たら遅れ（撮ってからの時間）を補正して更新 │
│ 行動（探す・追う・距離を保つ・撮る）→ コマンド      │
└────────────────────────────────────────────┘
```

- 周期は PiCar-X のハードに合わせている

  | もの | 周期 | 根拠 |
  | --- | --- | --- |
  | ハードへの書き込み | 50 Hz | サーボの PWM が 50 Hz（20 ms ごとにしか新しい角度を受け取らない）。I2C は 1 回 0.45 ms、全部書いて 3 ms ほど |
  | 制御ループ（推定と制御則） | 1 kHz | 書き込みの 20 倍。5 kHz（`rate_hz = 5000`）でも回るが、1 コアを 100 % 使う（1 kHz は 2 割ほど） |
  | カメラ・検出 | 60 fps | OV5647 の 640x480（このモードは 62.5 fps まで）。往復 16〜19 ms（サーバーで 8 ms）で、その間はループの予測で埋める。暗いときは露光が 16.6 ms で頭打ちになるので、ゲインを 16 まで上げる（AE は 8 まで） |
  | 超音波 | 15 Hz | 反響が消えるまで 60 ms あける |
  | モーターの PWM | 約 1.76 kHz | Robot HAT のクロック 72 MHz / prescaler 10 / period 4096（picarx の設定） |

- 速さの上限は 30（picarx の 0..100、PWM のデューティ 65 %）。TT モーターと後輪の径から、およそ 0.3〜0.4 m/s（実測していない）
- 方位 = 撮ったときのカメラの向き + 画面の中のずれ。距離 = `distance_k / 箱の高さ`（猫の高さ 25 cm を仮定）。
  snoo がほぼ正面にいて、超音波の値が近いときは超音波で補正する
- 追い始めたら、その画像と箱（猫らしさ・snoo らしさ）を `~/captures/snoo_seen/seen_日時.jpg/.json` に残す（10 秒に 1 枚まで）。本当に snoo を追ったかを後から確かめられる
- snoo を怖がらせない決まり（`guard.py`、走る行動の出力に最後にかぶせる）: ストレス（サーバーの `play.p.stress`）が高ければ止まって 25 cm 下がり、落ち着くまで待つ。自分が動いて 30 cm より近づかない。近づいた直後に snoo が速く離れたらストレスとみなす。1 回の遊びは 3 分、そのあと 10 分休む
- 電池が 6.4 V より低いのが 10 s 続いたら、止めて Pi を shutdown する（Pi も車の電池から給電していると、切れたときに電源ごと落ちて SD カードが壊れる）。
  一度だけ `sudo install -m 440 deploy/sudoers-snoo-shutdown /etc/sudoers.d/020_snoo_shutdown && sudo visudo -c` が要る
- 安全: 見失ったら止まる。制御プロセスが止まったらモーターを止める。速さの上限は既定 30（0..100）。超音波では止まらない（当てにならないので、snoo の距離の補正と地図の弱い証拠にだけ使う）

部屋を撮った画像は、撮ったときのカメラの向きから 1 枚のパノラマ（横=方位・縦=仰角、10° ごとの目盛り）にできる。
チルトのずれ（指令 0° で実際は +26.5°）とパンの量（指令の 0.96 倍）は home-linux の VGGT で測った値を既定にしている。

```sh
.venv/bin/python -m snoo_trucar panorama            # いちばん新しい map_* → その中の panorama.jpg
```

間取り（占有格子 room.json）は home-linux 側で、何か所かで撮った画像から作る。

## 入れる（85pi4）

```sh
git clone … ~/snoo_trucar
~/snoo_trucar/deploy/install.sh
sudo loginctl enable-linger $USER     # 電源を入れたら（ログインしなくても）動かす
```

- カメラ（PiCar-X 付属の OV5647）: `/boot/firmware/config.txt` を `camera_auto_detect=1` にする（IMX519 の `dtoverlay=imx519` は外す）。
  libcamera は IMX519 の AF のため Arducam 版が入っているので、`apt-mark hold` で上げないようにしてある
- 設定: `~/.config/snoo_trucar/config.toml`（例は `deploy/config.example.toml`、全項目は `snoo_trucar/config.py`）
- サーボのずれの較正値は `~/.config/snoo_trucar/picar-x.conf`（picarx の形式）

## 開発

```sh
.venv/bin/python -m pytest -q
.venv/bin/python -m snoo_trucar fake-server &                       # 偽の検出サーバー
.venv/bin/python -m snoo_trucar probe --image cat.jpg --server 127.0.0.1:8710
```
