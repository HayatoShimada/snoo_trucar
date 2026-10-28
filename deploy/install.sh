#!/usr/bin/env bash
# 85pi4 に入れる: venv（apt の picamera2 を使うため --system-site-packages）、robot-hat / picarx、
# 設定ファイル、systemd のユーザーサービス
set -euo pipefail
cd "$(dirname "$0")/.."

python3 -m venv --system-site-packages .venv
.venv/bin/pip install -q \
    "git+https://github.com/sunfounder/robot-hat.git@v2.0" \
    "git+https://github.com/sunfounder/picar-x.git@v2.0" \
    pytest
# robot_hat は import 時に pyaudio（音）を読む。snoo_trucar は音を使わないので空のものを置く
# （本物は portaudio の開発用ヘッダーが要り、pip では入らない）
SP=$(.venv/bin/python -c "import site; print(site.getsitepackages()[0])")
if ! .venv/bin/python -c "import pyaudio" 2>/dev/null; then
    printf '# robot_hat.music が import するだけ。snoo_trucar は音を使わない\npaInt16 = 8\n' > "$SP/pyaudio.py"
fi

mkdir -p ~/.config/snoo_trucar ~/.config/systemd/user
[ -f ~/.config/snoo_trucar/config.toml ] || cp deploy/config.example.toml ~/.config/snoo_trucar/config.toml
cp deploy/snoo-trucar.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable snoo-trucar
echo "入れた。起動: systemctl --user start snoo-trucar（電源を入れたときに動かすには sudo loginctl enable-linger $USER）"
