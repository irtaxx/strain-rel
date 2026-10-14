#!/usr/bin/env python3
"""
inspect_raw.py — memeriksa hasil rekaman Tahap 1

Jalankan di laptop (bukan di Pi) setelah menyalin file .i32 ke sana.
Keluarannya: ringkasan angka di terminal + satu file PNG berisi grafik.

Yang dicari di tahap ini ada tiga:
  1. Berapa besar derau saat tidak ada kereta  -> dasar penentuan ambang
  2. Seberapa tinggi puncak saat kereta lewat  -> apakah sinyalnya memadai
  3. Berapa lama satu lintasan di lokasi ini   -> panjang jendela perekaman

Pemakaian:
    python3 inspect_raw.py data/2026-10-14/raw_2026-10-14_09.i32
    python3 inspect_raw.py raw.i32 --dari 120 --sampai 180   # potong 1 menit
"""

import argparse
import json
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib belum terpasang.  Jalankan:  pip3 install matplotlib")

VREF_V = 2.5
GAUGE_FACTOR = 2.0        # ganti sesuai spesifikasi strain gauge Anda
BRIDGE = 0.25             # 0.25 = quarter bridge


def counts_to_microstrain(counts, gain):
    """Ubah hitungan ADC jadi microstrain (perkiraan, untuk gambaran saja)."""
    volt = counts * (2.0 * VREF_V / gain) / 0x7FFFFF
    # Vout/Vex = (GF * strain) * BRIDGE  ->  strain = Vout/Vex / (GF*BRIDGE)
    # Vex diasumsikan sama dengan VREF; sesuaikan kalau rangkaian Anda beda.
    return volt / VREF_V / (GAUGE_FACTOR * BRIDGE) * 1e6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", help="berkas .i32 hasil record_raw.py")
    ap.add_argument("--fs", type=int, default=0, help="laju cuplik (default: dari .json)")
    ap.add_argument("--gain", type=int, default=0, help="gain PGA (default: dari .json)")
    ap.add_argument("--dari", type=float, default=0, help="mulai detik ke-")
    ap.add_argument("--sampai", type=float, default=0, help="sampai detik ke- (0 = akhir)")
    ap.add_argument("--out", default="", help="nama file PNG keluaran")
    args = ap.parse_args()

    fs, gain = args.fs or 1000, args.gain or 64
    meta_path = args.file.replace(".i32", ".json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        fs = args.fs or meta.get("fs_hz", fs)
        gain = args.gain or meta.get("gain", gain)
        if meta.get("gaps"):
            print(f"Catatan: ada {len(meta['gaps'])} jeda data di berkas ini.")

    x = np.fromfile(args.file, dtype="<i4")
    if x.size == 0:
        sys.exit("Berkas kosong.")

    a = int(args.dari * fs)
    b = int(args.sampai * fs) if args.sampai else x.size
    x = x[a:b]
    t = np.arange(x.size) / fs

    ue = counts_to_microstrain(x.astype(np.float64), gain)

    # Baseline: median lebih tahan terhadap puncak dibanding rata-rata.
    base = np.median(ue)
    d = ue - base

    # Derau diukur dari bagian yang tenang saja. Memakai seluruh data akan
    # tercemar oleh event dan membuat ambang jadi terlalu tinggi.
    mad = np.median(np.abs(d))
    sigma = 1.4826 * mad              # perkiraan simpangan baku yang tahan pencilan
    quiet = np.abs(d) < 4 * sigma
    noise = d[quiet].std() if quiet.sum() > fs else sigma

    amb6 = 6 * noise
    over = np.abs(d) > amb6

    print(f"\n{'=' * 58}")
    print(f"  {os.path.basename(args.file)}")
    print(f"{'=' * 58}")
    print(f"  Durasi                : {x.size/fs/60:.1f} menit  ({x.size:,} sampel)"
          .replace(",", "."))
    print(f"  Laju cuplik           : {fs} Hz,  gain {gain}")
    print(f"  Baseline              : {base:,.1f} µε".replace(",", "."))
    print(f"  Derau saat tenang     : {noise:.2f} µε rms")
    print(f"  Simpangan maksimum    : {np.abs(d).max():,.1f} µε".replace(",", "."))
    print(f"  Rasio sinyal/derau    : {np.abs(d).max()/noise:,.0f}×".replace(",", "."))
    print(f"\n  Di atas 6× derau      : {over.sum()/fs:.1f} detik "
          f"({over.sum()/x.size*100:.2f}% dari rekaman)")
    print(f"\n  >> Usulan ambang trigger: {amb6:,.1f} µε dari baseline"
          .replace(",", "."))
    print(f"     (6× derau — cukup longgar untuk tidak terpicu angin "
          f"atau getaran kecil)")

    if np.abs(d).max() < 20 * noise:
        print("\n  PERINGATAN: simpangan terbesar kurang dari 20× derau.")
        print("  Kemungkinan belum ada kereta lewat selama perekaman, atau")
        print("  penguatan jembatan terlalu kecil. Rekam lagi lebih lama.")

    # ---------- Grafik ----------
    fig, ax = plt.subplots(3, 1, figsize=(13, 10))
    fig.suptitle(os.path.basename(args.file), fontsize=13, fontweight="bold")
    GREEN, GOLD, RED = "#0D7254", "#B8860B", "#B3261E"

    # 1. seluruh rekaman (diperjarang agar ringan digambar)
    step = max(1, x.size // 200_000)
    ax[0].plot(t[::step] / 60, d[::step], color=GREEN, lw=.5)
    ax[0].axhline(amb6, color=GOLD, ls="--", lw=1, label=f"ambang 6×derau = {amb6:.0f} µε")
    ax[0].axhline(-amb6, color=GOLD, ls="--", lw=1)
    ax[0].set_xlabel("menit")
    ax[0].set_ylabel("µε dari baseline")
    ax[0].set_title("Seluruh rekaman", loc="left", fontsize=10)
    ax[0].legend(fontsize=8, loc="upper right")
    ax[0].grid(alpha=.3)

    # 2. jendela di sekitar simpangan terbesar
    c = int(np.argmax(np.abs(d)))
    w = int(10 * fs)
    s, e = max(0, c - w), min(x.size, c + w)
    ax[1].plot(t[s:e], d[s:e], color=GREEN, lw=.8)
    ax[1].axhline(amb6, color=GOLD, ls="--", lw=1)
    ax[1].axhline(-amb6, color=GOLD, ls="--", lw=1)
    ax[1].set_xlabel("detik")
    ax[1].set_ylabel("µε dari baseline")
    ax[1].set_title("Sekitar simpangan terbesar — apakah ini kereta lewat?",
                    loc="left", fontsize=10)
    ax[1].grid(alpha=.3)

    # 3. perbesaran 1,5 detik: di sini pola gandar harus mulai terlihat
    w2 = int(.75 * fs)
    s2, e2 = max(0, c - w2), min(x.size, c + w2)
    ax[2].plot(t[s2:e2], d[s2:e2], color=RED, lw=1.1)
    ax[2].set_xlabel("detik")
    ax[2].set_ylabel("µε dari baseline")
    ax[2].set_title("Perbesaran 1,5 detik — pola gandar seharusnya terbaca di sini",
                    loc="left", fontsize=10)
    ax[2].grid(alpha=.3)

    plt.tight_layout(rect=[0, 0, 1, .97])
    out = args.out or args.file.replace(".i32", "_periksa.png")
    fig.savefig(out, dpi=120)
    print(f"\n  Grafik tersimpan: {out}\n")


if __name__ == "__main__":
    main()
