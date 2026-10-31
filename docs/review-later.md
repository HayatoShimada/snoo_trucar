# レビュー（snoo_trucar#21）で後に回したもの

review/safety-fixes で A・B・C・1・5・6・G・H と、E・F の一部を直した。#26 で Pi だけの探索と超音波で止める所を
外したので、D と F の残りは無くなった。#35 で E・7・I・4 とリファクタを直した。残りと、直すならどう変えるか。

## 直した（#35）
- **E の残り：前に進む短い 1 歩では詰まりの検知が働かない**：motion.py の MotionWatch が、走ったひと続きごとに
  走り出して 0.4 s 後の画像を基にした差の最大を覚え、止まったときにざらつきと変わらなければ「動いていなかった」とする。
  station.py が prev_step に `image_motion` として付け、前へ走ったひと続きが動いていなかったのに stop_reason が
  done なら stuck にする（制御プロセスの最短 2.2 s の検知はそのまま）
- **7・I：clear.jsonl**：開いたままにして 1 s ごとに flush する（最大 60 回/s で開いて閉じていた）。finish で閉じる
- **4：runner の history**：シミュレーションのときだけ溜める
- **リファクタ**：`Controller._explore` を段階ごとに分けた（撮る `_scan_tick`・返事を待つ `_explore_plan`・走る `_step`）。
  cmd の 18 から後を control_proc の名前付きの番号（`C_STATION_WAIT` など）にした

## 残り
- 電池の判定に幅が無い：止めるのは battery_low_v（6.6 V）未満が 3 回続いたとき、走り出すのは 6.8 V より上。
- 走っている間の 5 fps の画像と frames.jsonl も SD に書く（画像は station に送るので要る。frames.jsonl は
  起動し直したときの続き（#30）に使う）
- **9. shm のメモリバリア**：優先度低。CPython の numpy の代入は、書き手 1・読み手 1 では今のところ問題が出ていない
