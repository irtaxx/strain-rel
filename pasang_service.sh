#!/bin/bash
# ======================================================================
#  pasang_service.sh — membuat strain_agent.py jalan otomatis saat boot
#
#  Jalankan sekali saja di Raspberry Pi:
#      bash pasang_service.sh
#
#  Script ini mencari sendiri letak strain_agent.py dan site.json, jadi
#  tidak masalah apakah berkasnya ada di ~/strain-rel/pi/ atau langsung
#  di ~/strain-rel/.
# ======================================================================
set -e

NAMA="strain-agent"
PENGGUNA="$(whoami)"
RUMAH="$HOME"

echo "=== Memasang layanan $NAMA untuk pengguna $PENGGUNA ==="
echo

# ---------- 1. Cari strain_agent.py ----------
SKRIP=""
for kandidat in \
    "$RUMAH/strain-rel/pi/strain_agent.py" \
    "$RUMAH/strain-rel/strain_agent.py" \
    "$RUMAH/strain_agent.py"
do
    [ -f "$kandidat" ] && SKRIP="$kandidat" && break
done

if [ -z "$SKRIP" ]; then
    echo "GAGAL: strain_agent.py tidak ditemukan."
    echo "Dicari di:"
    echo "  $RUMAH/strain-rel/pi/"
    echo "  $RUMAH/strain-rel/"
    echo "  $RUMAH/"
    exit 1
fi

FOLDER="$(dirname "$SKRIP")"
echo "Skrip   : $SKRIP"

# ---------- 2. Pastikan site.json ada ----------
if [ ! -f "$FOLDER/site.json" ]; then
    echo
    echo "GAGAL: $FOLDER/site.json belum ada."
    if [ -f "$FOLDER/site.json.example" ]; then
        echo "Buat dulu dengan:"
        echo "  cp $FOLDER/site.json.example $FOLDER/site.json"
        echo "  nano $FOLDER/site.json"
    fi
    exit 1
fi
echo "Config  : $FOLDER/site.json"

# ---------- 3. Pastikan port serial terbaca ----------
PORT="$(python3 - "$FOLDER/site.json" <<'PY'
import json, sys
print(json.load(open(sys.argv[1])).get("port", "/dev/ttyUSB0"))
PY
)"
echo "Port    : $PORT"

if [ ! -e "$PORT" ]; then
    echo
    echo "PERINGATAN: $PORT tidak ada sekarang."
    echo "Layanan tetap dipasang — systemd akan mencoba ulang tiap 10 detik"
    echo "sampai ESP32 tercolok. Tapi periksa dulu dengan: ls /dev/ttyUSB*"
fi

if ! groups | grep -qw dialout; then
    echo
    echo "PERINGATAN: pengguna $PENGGUNA belum masuk grup dialout."
    echo "Jalankan:  sudo usermod -a -G dialout $PENGGUNA  lalu reboot."
fi

# ---------- 4. Hentikan tmux kalau sedang jalan ----------
# Dua proses tidak bisa memakai satu port serial bersamaan. Kalau tmux
# dibiarkan hidup, layanan akan gagal terus dengan pesan "port sibuk"
# yang membingungkan karena sumbernya tidak kelihatan.
if pgrep -f "strain_agent.py" > /dev/null; then
    echo
    echo "Ada strain_agent.py yang sedang berjalan (kemungkinan di tmux)."
    echo "Dihentikan dulu supaya tidak berebut port serial."
    pkill -f "strain_agent.py" || true
    sleep 2
fi

# ---------- 5. Tulis unit systemd ----------
echo
echo "Menulis /etc/systemd/system/$NAMA.service ..."

sudo tee "/etc/systemd/system/$NAMA.service" > /dev/null <<EOF
[Unit]
Description=Strain gauge agent - deteksi lintasan kereta
Documentation=https://github.com/irtaxx/strain-rel
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$PENGGUNA
WorkingDirectory=$FOLDER
ExecStart=/usr/bin/python3 $SKRIP --config $FOLDER/site.json

# Tanpa ini, Python menahan keluarannya di penyangga saat tidak berjalan
# di terminal, dan journalctl tampak kosong berjam-jam meski programnya
# sehat. Gejala yang sangat membingungkan kalau tidak tahu sebabnya.
Environment=PYTHONUNBUFFERED=1

Restart=always
RestartSec=10

# Kalau gagal terus (misal ESP32 tidak pernah tercolok), jangan menyerah
# permanen — di lapangan tidak ada yang bisa menyalakannya kembali.
StartLimitIntervalSec=0

StandardOutput=journal
StandardError=journal
SyslogIdentifier=$NAMA

[Install]
WantedBy=multi-user.target
EOF

# ---------- 6. Nyalakan ----------
sudo systemctl daemon-reload
sudo systemctl enable "$NAMA"
sudo systemctl restart "$NAMA"

sleep 3

echo
echo "=== Status ==="
sudo systemctl status "$NAMA" --no-pager -l | head -20

echo
echo "======================================================================"
echo " Selesai. Layanan akan ikut hidup setiap kali Pi dinyalakan."
echo
echo " Perintah sehari-hari:"
echo "   bash $FOLDER/cek.sh              lihat ringkasan"
echo "   journalctl -u $NAMA -f           lihat keluaran langsung"
echo "   sudo systemctl restart $NAMA     mulai ulang"
echo "   sudo systemctl stop $NAMA        hentikan sementara"
echo "======================================================================"
