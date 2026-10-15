#!/bin/bash
# ======================================================================
#  cek.sh — ringkasan keadaan unit strain gauge
#
#      bash cek.sh
#
#  Dibuat untuk dibaca lewat remote shell: satu layar, tanpa perlu
#  mengingat perintah systemd.
# ======================================================================

NAMA="strain-agent"
H='\033[1m'; N='\033[0m'
HIJAU='\033[0;32m'; MERAH='\033[0;31m'; KUNING='\033[0;33m'

# Cari folder kerja dari unit systemd; kalau belum terpasang, tebak.
FOLDER="$(systemctl show "$NAMA" -p WorkingDirectory --value 2>/dev/null)"
[ -z "$FOLDER" ] && FOLDER="$HOME/strain-rel/pi"
[ -d "$FOLDER" ] || FOLDER="$HOME/strain-rel"

echo
echo -e "${H}═══ UNIT STRAIN GAUGE ═══${N}"
echo "  $(date '+%A, %d %B %Y, %H:%M:%S')   uptime$(uptime -p | sed 's/up//')"

# ---------- 1. Layanan ----------
echo
echo -e "${H}Layanan${N}"
if ! systemctl list-unit-files | grep -q "^$NAMA.service"; then
    echo -e "  ${MERAH}● belum dipasang${N}  — jalankan: bash pasang_service.sh"
else
    AKTIF="$(systemctl is-active $NAMA)"
    BOOT="$(systemctl is-enabled $NAMA 2>/dev/null)"
    if [ "$AKTIF" = "active" ]; then
        SEJAK="$(systemctl show $NAMA -p ActiveEnterTimestamp --value)"
        PID="$(systemctl show $NAMA -p MainPID --value)"
        RAM="$(ps -o rss= -p "$PID" 2>/dev/null | awk '{printf "%.0f MB", $1/1024}')"
        echo -e "  ${HIJAU}● berjalan${N}   sejak $SEJAK"
        echo "    pid $PID   memori ${RAM:-?}"
    else
        echo -e "  ${MERAH}● $AKTIF${N}"
        systemctl show $NAMA -p ExecMainStatus -p NRestarts --value 2>/dev/null \
          | paste -sd' ' - | sed 's/^/    kode keluar & jumlah restart: /'
    fi
    if [ "$BOOT" = "enabled" ]; then
        echo -e "    ${HIJAU}✓${N} ikut hidup saat Pi dinyalakan"
    else
        echo -e "    ${KUNING}!${N} TIDAK ikut hidup saat boot — jalankan: sudo systemctl enable $NAMA"
    fi
fi

# ---------- 2. ESP32 ----------
echo
echo -e "${H}Sambungan ESP32${N}"
PORTS="$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)"
if [ -z "$PORTS" ]; then
    echo -e "  ${MERAH}● tidak terdeteksi${N}  — periksa kabel USB (banyak kabel murah tanpa jalur data)"
else
    for p in $PORTS; do echo -e "  ${HIJAU}●${N} $p"; done
fi

# ---------- 3. Data ----------
echo
echo -e "${H}Data${N}"
DIRDATA="$HOME/strain-data"
if [ -d "$DIRDATA" ]; then
    HARI="$(date +%F)"
    MENTAH="$DIRDATA/$HARI"
    if [ -d "$MENTAH" ]; then
        UKURAN="$(du -sh "$MENTAH" 2>/dev/null | cut -f1)"
        BARU="$(ls -t "$MENTAH"/*.i32 2>/dev/null | head -1)"
        if [ -n "$BARU" ]; then
            LALU=$(( $(date +%s) - $(stat -c %Y "$BARU") ))
            if [ "$LALU" -lt 120 ]; then
                echo -e "  ${HIJAU}●${N} arsip mentah hari ini: $UKURAN, ditulis ${LALU}s lalu"
            else
                echo -e "  ${MERAH}●${N} arsip mentah tidak bertambah selama ${LALU}s — data mungkin berhenti"
            fi
        fi
    else
        echo -e "  ${KUNING}!${N} belum ada arsip mentah hari ini"
    fi

    EV="$(ls "$DIRDATA/events"/*.json 2>/dev/null | wc -l)"
    EVHARI="$(ls "$DIRDATA/events"/ev_$(date +%Y%m%d)_*.json 2>/dev/null | wc -l)"
    echo "  event tercatat : $EV total, $EVHARI hari ini"

    ANTRE="$(ls "$DIRDATA/antrean"/*.json 2>/dev/null | wc -l)"
    if [ "$ANTRE" -eq 0 ]; then
        echo -e "  antrean kirim  : ${HIJAU}kosong${N} — semua sudah sampai ke web"
    else
        echo -e "  antrean kirim  : ${KUNING}$ANTRE menunggu${N} — jaringan sedang bermasalah?"
    fi

    EVBARU="$(ls -t "$DIRDATA/events"/*.json 2>/dev/null | head -1)"
    [ -n "$EVBARU" ] && echo "  event terakhir : $(date -r "$EVBARU" '+%d %b %H:%M:%S')"

    TOTAL="$(du -sh "$DIRDATA" 2>/dev/null | cut -f1)"
    NHARI="$(ls -d "$DIRDATA"/20??-??-?? 2>/dev/null | wc -l)"
    SIMPAN="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('simpan_mentah_hari',14))" "$FOLDER/site.json" 2>/dev/null)"
    echo "  arsip tersimpan: ${TOTAL:-?} dalam $NHARI hari${SIMPAN:+, batas $SIMPAN hari}"
else
    echo -e "  ${KUNING}!${N} folder $DIRDATA belum ada"
fi

# ---------- 4. Sistem ----------
echo
echo -e "${H}Sistem${N}"
SISA="$(df -h / | awk 'NR==2{print $4" bebas dari "$2" ("$5" terpakai)"}')"
PERSEN="$(df / | awk 'NR==2{gsub("%","",$5); print $5}')"
if [ "$PERSEN" -gt 85 ]; then
    echo -e "  ${MERAH}●${N} kartu SD: $SISA"
else
    echo -e "  ${HIJAU}●${N} kartu SD: $SISA"
fi
SUHU="$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2)"
[ -z "$SUHU" ] && SUHU="$(awk '{printf "%.1f'"'"'C", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)"
echo "  suhu CPU : ${SUHU:-tidak terbaca}"
if command -v timedatectl > /dev/null; then
    NTP="$(timedatectl show -p NTPSynchronized --value 2>/dev/null)"
    [ "$NTP" = "yes" ] \
      && echo -e "  jam      : ${HIJAU}tersinkron NTP${N}" \
      || echo -e "  jam      : ${KUNING}BELUM tersinkron${N} — penanda waktu event bisa salah"
fi

# ---------- 5. Keluaran terakhir ----------
echo
echo -e "${H}10 baris terakhir${N}"
journalctl -u "$NAMA" -n 10 --no-pager -o cat 2>/dev/null | sed 's/^/  /' \
  || echo "  (belum ada)"

echo
echo "  journalctl -u $NAMA -f     untuk melihat terus"
echo
