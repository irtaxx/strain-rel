#!/usr/bin/env python3
"""
record_raw.py — TAHAP 1: perekam data mentah di Raspberry Pi

Membaca bingkai biner dari ESP32 lewat USB, memeriksa keutuhannya, lalu
menulis apa adanya ke file. Belum ada deteksi kereta — tahap ini memang
hanya untuk mengumpulkan data nyata, supaya ambang trigger nanti
ditentukan dari angka lapangan, bukan dari tebakan.

File ditulis per jam:
    data/2026-10-14/raw_2026-10-14_09.i32    sampel int32 mentah, beruntun
    data/2026-10-14/raw_2026-10-14_09.json   keterangan: fs, gain, jeda data

Pemakaian:
    python3 record_raw.py
    python3 record_raw.py --port /dev/ttyUSB0 --dir /home/pi/strain-data
    python3 record_raw.py --durasi 3600          # berhenti sendiri setelah 1 jam

Tekan Ctrl-C untuk berhenti dengan rapi.
"""

import argparse
import json
import os
import signal
import struct
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial belum terpasang.  Jalankan:  pip3 install pyserial")

MAGIC = b"\xA5\x5A"
HEADER = 16          # magic2 + ver1 + n1 + seq4 + t_us8
VERSION = 1

# Dipakai untuk mengubah hitungan ADC jadi microstrain nanti. Belum
# dipakai di tahap ini, tapi ikut dicatat supaya file lama tetap bisa
# ditafsirkan kalau pengaturannya berubah di kemudian hari.
VREF_V = 2.5
GAIN = 64
ADC_FULLSCALE = 0x7FFFFF


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, sama persis dengan yang di ESP32."""
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


class HourlyWriter:
    """Menulis sampel ke file per jam, lengkap dengan berkas keterangan."""

    def __init__(self, root, fs):
        self.root = root
        self.fs = fs
        self.key = None
        self.f = None
        self.path = None
        self.n = 0
        self.gaps = []

    def _close(self):
        if self.f:
            self.f.close()
            meta = {
                "file": os.path.basename(self.path),
                "fs_hz": self.fs,
                "dtype": "int32-le",
                "vref_v": VREF_V,
                "gain": GAIN,
                "adc_fullscale": ADC_FULLSCALE,
                "samples": self.n,
                "gaps": self.gaps,
                "closed_at": datetime.now().isoformat(timespec="seconds"),
            }
            with open(self.path.replace(".i32", ".json"), "w") as m:
                json.dump(meta, m, indent=2)
            print(f"\n  tersimpan: {self.path}  ({self.n:,} sampel, "
                  f"{len(self.gaps)} jeda)".replace(",", "."))
            self.f = None

    def write(self, blob, nsamp):
        now = datetime.now()
        key = now.strftime("%Y-%m-%d_%H")
        if key != self.key:
            self._close()
            day = now.strftime("%Y-%m-%d")
            d = os.path.join(self.root, day)
            os.makedirs(d, exist_ok=True)
            self.path = os.path.join(d, f"raw_{key}.i32")
            self.f = open(self.path, "ab", buffering=1 << 16)
            self.key = key
            self.n = 0
            self.gaps = []
            print(f"\n  file baru: {self.path}")
        self.f.write(blob)
        self.n += nsamp

    def note_gap(self, missing_samples):
        self.gaps.append({"at_sample": self.n, "missing": missing_samples})

    def close(self):
        self._close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="port serial ESP32 (lihat: ls /dev/ttyUSB* /dev/ttyACM*)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--fs", type=int, default=1000, help="laju cuplik, Hz")
    ap.add_argument("--dir", default="data", help="folder penyimpanan")
    ap.add_argument("--durasi", type=float, default=0,
                    help="berhenti otomatis setelah sekian detik (0 = terus)")
    args = ap.parse_args()

    print(f"Membuka {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Gagal membuka port: {e}\n"
                 f"Cek dengan: ls /dev/ttyUSB* /dev/ttyACM*")

    writer = HourlyWriter(args.dir, args.fs)
    running = {"on": True}

    def stop(*_):
        running["on"] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    buf = bytearray()
    prev_seq = None
    t_start = time.time()
    t_tick = t_start

    # Statistik per detik
    win = []
    stat = {"frames": 0, "bad_crc": 0, "lost": 0, "samples": 0}

    print("Merekam. Ctrl-C untuk berhenti.\n")
    print(f"{'waktu':>8} {'laju':>9} {'rerata':>12} {'derau rms':>11} "
          f"{'min':>12} {'maks':>12} {'rusak':>6} {'hilang':>7}")

    while running["on"]:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)

        # Urai semua bingkai lengkap yang ada di penyangga
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                # Tidak ada magic sama sekali; buang sisa agar tidak menumpuk,
                # sisakan 1 byte kalau-kalau magic terpotong di batas baca.
                if len(buf) > 2:
                    del buf[:-1]
                break
            if i > 0:
                # Byte sebelum magic adalah sampah atau baris '#' dari ESP32
                junk = bytes(buf[:i])
                if junk.strip().startswith(b"#"):
                    for line in junk.split(b"\n"):
                        if line.strip():
                            print("  ESP32:", line.decode(errors="replace").strip())
                del buf[:i]

            if len(buf) < HEADER:
                break
            ver = buf[2]
            n = buf[3]
            total = HEADER + 4 * n + 2
            if ver != VERSION or n == 0 or n > 200:
                del buf[:2]          # magic palsu, geser dan cari lagi
                continue
            if len(buf) < total:
                break

            frame = bytes(buf[:total])
            del buf[:total]

            got = struct.unpack_from("<H", frame, total - 2)[0]
            if got != crc16(frame[2:total - 2]):
                stat["bad_crc"] += 1
                continue

            seq = struct.unpack_from("<I", frame, 4)[0]
            if prev_seq is not None and seq != prev_seq + 1:
                missing = (seq - prev_seq - 1) * n
                if 0 < missing < 10_000_000:
                    stat["lost"] += missing
                    writer.note_gap(missing)
            prev_seq = seq

            payload = frame[HEADER:HEADER + 4 * n]
            writer.write(payload, n)

            stat["frames"] += 1
            stat["samples"] += n
            win.extend(struct.unpack(f"<{n}i", payload))

        # Laporan tiap detik
        now = time.time()
        if now - t_tick >= 1.0:
            dt = now - t_tick
            t_tick = now
            if win:
                m = sum(win) / len(win)
                var = sum((v - m) ** 2 for v in win) / len(win)
                rms = var ** 0.5
                print(f"{time.strftime('%H:%M:%S'):>8} "
                      f"{len(win)/dt:8.1f}/s "
                      f"{m:12.0f} {rms:11.0f} "
                      f"{min(win):12.0f} {max(win):12.0f} "
                      f"{stat['bad_crc']:6d} {stat['lost']:7d}")
            else:
                print(f"{time.strftime('%H:%M:%S'):>8} "
                      f"{'tidak ada data':>9} "
                      "— periksa kabel USB dan apakah ESP32 menyala")
            win = []

        if args.durasi and (now - t_start) >= args.durasi:
            break

    writer.close()
    ser.close()
    dur = time.time() - t_start
    print(f"\nSelesai. {stat['samples']:,} sampel dalam {dur/60:.1f} menit."
          .replace(",", "."))
    print(f"Bingkai rusak: {stat['bad_crc']}   Sampel hilang: {stat['lost']}")
    if stat["bad_crc"] > stat["frames"] * 0.001:
        print("\nBingkai rusak lebih dari 0,1% — biasanya kabel USB terlalu "
              "panjang atau terganggu. Coba kabel lebih pendek atau berpelindung.")


if __name__ == "__main__":
    main()
