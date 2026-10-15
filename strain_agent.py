#!/usr/bin/env python3
"""
strain_agent.py — TAHAP 2: program lapangan di Raspberry Pi

Menggantikan record_raw.py. Satu program, empat pekerjaan sekaligus:

  1. Membaca aliran bingkai dari ESP32 (1000 sampel/detik)
  2. Melacak baseline, lalu mendeteksi "ada sesuatu lewat"
  3. Memotong waveform event, menghitung fiturnya, menyimpannya
  4. Mengirim ke web; kalau jaringan putus, event mengantre di kartu SD

Arsip mentah 1000 Hz tetap ditulis seperti sebelumnya, supaya kalau
algoritma deteksi nanti diperbaiki, data lama bisa diolah ulang.

Pemakaian:
    python3 strain_agent.py                      # pakai site.json
    python3 strain_agent.py --no-upload          # uji lokal, tanpa kirim
    python3 strain_agent.py --ambang 9300        # paksa ambang tertentu

Tekan Ctrl-C untuk berhenti dengan rapi.
"""

import argparse
import base64
import json
import os
import re
import signal
import struct
import sys
import threading
import time
import urllib.error
import urllib.request
import zlib
from collections import deque
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial belum terpasang:  pip3 install pyserial --break-system-packages")

MAGIC = b"\xA5\x5A"
HEADER = 16
VERSION = 1


# ======================================================================
# Pengaturan
# ======================================================================

def muat_konfigurasi(path):
    """
    site.json berisi identitas unit dan lokasinya. Berkas ini ikut
    menentukan isi tiap event yang dikirim, sehingga tiap baris data
    membawa keterangannya sendiri — penting karena unit berpindah-pindah.
    """
    bawaan = {
        "unit_id": "unit-01",
        "deployment_id": 1,
        "lokasi": "belum diisi",
        "port": "/dev/ttyUSB0",
        "baud": 921600,
        "fs_hz": 1000,
        "gain": 64,
        "bridge_n": 1,
        "gauge_factor": 2.31,
        "v_excitation": 3.3,
        "vref": 2.048,

        # --- Deteksi event ---
        # Ambang dalam hitungan ADC. Dari uji meja: derau 1.550, jadi
        # 6x derau = 9.300. Setel ulang setelah ada data lapangan.
        "ambang_hitungan": 9300,
        "pre_detik": 1.5,        # berapa lama sebelum trigger ikut disimpan
        "hold_off_detik": 2.0,   # diam selama ini baru event dinyatakan selesai
        "maks_durasi_detik": 60, # pengaman kalau sensor rusak dan nilainya terkunci
        "min_durasi_detik": 0.3, # lebih pendek dari ini dianggap gangguan, bukan event
        "baseline_detik": 60,    # panjang rata-rata bergerak untuk baseline

        # --- Perhitungan kecepatan ---
        # Dikosongkan sampai jenis sarana yang melintas diketahui dari KAI.
        # 0 berarti kecepatan tidak dihitung — lebih baik kosong daripada
        # diisi angka yang diam-diam berdasar asumsi.
        "jarak_gandar_m": 0.0,
        # Lembah yang saling bertindih tertarik saling mendekat, sehingga
        # jarak terukur lebih pendek dari jarak sebenarnya dan kecepatan
        # terhitung terlalu tinggi — pada uji sintetis sekitar 7%. Isi
        # faktor ini setelah dibandingkan dengan kecepatan kereta yang
        # diketahui dari KAI.
        "koreksi_kecepatan": 1.0,

        # --- Pengiriman ---
        "url_event": "https://kaimonitoringiot.com/api/strain-event",
        "api_key": "",
        "arsip_mentah": True,
        "dir_data": "~/strain-data",
    }
    if os.path.exists(path):
        with open(path) as f:
            bawaan.update(json.load(f))
    else:
        print(f"  {path} tidak ada — memakai pengaturan bawaan.")
        print(f"  Salin site.json.example jadi site.json lalu sesuaikan.\n")
    bawaan["dir_data"] = os.path.expanduser(bawaan["dir_data"])
    return bawaan


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


# ======================================================================
# Pelacak baseline
# ======================================================================

class Baseline:
    """
    Rata-rata bergerak sederhana, DIBEKUKAN selama event berlangsung.

    Pembekuan itu intinya. Kalau rata-ratanya terus berjalan saat kereta
    lewat, dia akan ikut tertarik mengikuti puncak, dan amplitudo event
    jadi terhitung lebih kecil daripada yang sebenarnya.
    """

    def __init__(self, fs, detik):
        self.n = max(1, int(fs * detik))
        self.buf = deque(maxlen=self.n)
        self.jumlah = 0.0
        self.beku = False

    def reset(self):
        """
        Lupakan semuanya dan pelajari ulang dari nol.

        Dipakai setelah event ditutup paksa karena kepanjangan. Baseline
        saat itu sudah dibekukan berpuluh detik dan kemungkinan besar
        sudah jauh dari level sebenarnya. Kalau dipakai lagi apa adanya,
        simpangannya tetap besar, event langsung terpicu lagi, dan
        baseline membeku lagi — lingkaran yang tidak bisa keluar sendiri.
        """
        self.buf.clear()
        self.jumlah = 0.0
        self.beku = False

    def tambah(self, v):
        if self.beku:
            return
        if len(self.buf) == self.n:
            self.jumlah -= self.buf[0]
        self.buf.append(v)
        self.jumlah += v

    @property
    def nilai(self):
        return self.jumlah / len(self.buf) if self.buf else 0.0

    def ambil_puncak(self):
        """Nilai tertinggi sejak panggilan terakhir, lalu dinolkan."""
        a, b = self.sta_puncak, self.rasio_puncak
        self.sta_puncak = self.rasio_puncak = 0.0
        return a, b

    @property
    def siap(self):
        # Sebelum penyangga terisi separuh, baseline belum bisa dipercaya
        # dan deteksi harus ditahan dulu.
        return len(self.buf) >= self.n // 2


# ======================================================================
# Pendeteksi event
# ======================================================================

class Detektor:
    """
    Deteksi berdasarkan AKTIVITAS, bukan posisi — STA/LTA.

    Rancangan lama bertanya "seberapa jauh nilai ini dari baseline?".
    Pertanyaan itu tidak bisa membedakan kereta dari rel yang memuai:
    keduanya memindahkan nilai, dan yang satu lambat sedangkan yang lain
    cepat. Begitu hanyut termal melewati ambang, deteksi menyala dan tidak
    pernah bisa mati lagi.

    Sekarang yang ditanyakan "seberapa ramai sinyal ini dibanding biasanya?":

      STA = simpangan baku dalam blok pendek (50 ms) — keramaian sesaat
      LTA = rata-rata STA selama 30 detik terakhir — keramaian biasanya
      rasio = STA / LTA

    Hanyut sebesar apa pun tidak menaikkan rasio, karena dalam 50 ms ia
    hampir tidak bergerak. Kereta menaikkannya puluhan kali lipat.

    Dua sifat yang ikut didapat:

    1. Rasio tidak bersatuan. Ambang 4x berlaku di akrilik maupun di baja,
       pada gain berapa pun. Unit yang berpindah lokasi tidak perlu
       dikalibrasi ulang.

    2. Pintu keluarnya selebar pintu masuk. Dulu masuk butuh 8 sampel dan
       keluar butuh 2.000 sampel berturut-turut tenang — timpang 250 kali,
       dan itulah sebabnya event terkunci. Rasio jatuh ke sepersepuluh
       begitu gerakan berhenti, jadi menutup event jadi mudah.

    LTA dibekukan selama event. Tanpa itu, kereta sendiri menaikkan LTA,
    rasionya mengecil, dan event tertutup di tengah lintasan.
    """

    def __init__(self, cfg, saat_event_selesai):
        fs = cfg["fs_hz"]
        self.cfg = cfg
        self.fs = fs

        sta_detik = float(cfg.get("sta_detik", 0.05))
        self.blok_n = max(1, int(fs * sta_detik))
        self.lta_blok = max(4, int(float(cfg.get("lta_detik", 30)) / sta_detik))
        self.rasio_picu = float(cfg.get("rasio_picu", 4.0))
        self.rasio_lepas = float(cfg.get("rasio_lepas", 2.0))

        # Lantai absolut. Rasio tidak bersatuan, jadi di lingkungan yang
        # sangat sunyi goyangan 3 µε pun bisa menghasilkan rasio 10x.
        # Lantai ini membuang hal-hal remeh; nilainya tidak perlu presisi,
        # cukup jauh di bawah kereta teringan.
        self.sta_minimum = float(cfg.get("sta_minimum", 3000))

        # Gerbang amplitudo. Dua penyaring di atas hanya menilai KERAMAIAN;
        # keduanya bisa dilewati lonjakan sempit yang datang rapat, betapapun
        # dangkalnya. Gerbang ini menilai KEDALAMAN: kalau titik terdalam
        # sepanjang event masih di bawah angka ini, apa pun yang terjadi tadi
        # terlalu kecil untuk disebut kereta.
        #
        # Dalam mikrostrain, bukan hitungan ADC, supaya angkanya tidak perlu
        # dihitung ulang setiap kali gain diubah — dan supaya sama dengan
        # kolom "Puncak" di tabel web.
        self.min_puncak_ue = float(cfg.get("min_puncak_ue", 0))

        self.pre = deque(maxlen=int(fs * cfg["pre_detik"]))
        self.hold_off_blok = max(1, int(fs * cfg["hold_off_detik"] / self.blok_n))
        self.maks_n = int(fs * cfg["maks_durasi_detik"])
        self.min_n = int(fs * cfg["min_durasi_detik"])
        self.min_blok_ramai = max(
            1, int(fs * float(cfg.get("min_lewat_detik", 0.15)) / self.blok_n))

        # Baseline ini TIDAK lagi dipakai untuk deteksi — hanya untuk kolom
        # tampilan dan untuk memperlihatkan hanyut. Karena itu ia tidak
        # pernah dibekukan, dan tidak bisa mengunci apa pun lagi.
        self.baseline = Baseline(fs, cfg["baseline_detik"])
        self.saat_selesai = saat_event_selesai

        self.blok = []
        self.lta = deque(maxlen=self.lta_blok)
        self.lta_jumlah = 0.0
        self.sta = 0.0
        self.rasio = 0.0

        # Puncak sejak laporan terakhir. Baris status dicetak tiap 2-3 detik
        # sementara blok diolah 20 kali per detik, jadi menampilkan blok
        # terakhir menyembunyikan justru lonjakan yang memicu event.
        self.sta_puncak = 0.0
        self.rasio_puncak = 0.0

        self.aktif = False
        self.sampel = []
        self.diam_blok = 0
        self.blok_ramai = 0
        self.base_saat_trigger = 0.0
        self.t_mulai = None
        self.jumlah_event = 0

        self.ditolak = 0            # event yang dibuang karena terlalu singkat
        self.tertahan = 0           # rasio naik tapi tidak lolos lantai absolut
        self.timeout_beruntun = 0
        self.puncak_tolak = 0.0

    @property
    def siap(self):
        """LTA sudah terisi penuh; sebelum itu tidak ada pembanding."""
        return len(self.lta) >= self.lta_blok

    def olah(self, nilai, t_wall):
        """Sekumpulan sampel masuk. t_wall = waktu dinding sampel terakhir."""
        for v in nilai:
            self.baseline.tambah(v)
            if self.aktif:
                self.sampel.append(v)
            else:
                self.pre.append(v)
            self.blok.append(v)
            if len(self.blok) >= self.blok_n:
                self._olah_blok(t_wall)

    def _olah_blok(self, t_wall):
        b, self.blok = self.blok, []
        n = len(b)
        m = sum(b) / n
        self.sta = (sum((v - m) ** 2 for v in b) / n) ** 0.5

        if not self.siap:
            self.lta.append(self.sta)
            self.lta_jumlah += self.sta
            return

        acuan = self.lta_jumlah / len(self.lta)
        self.rasio = (self.sta / acuan) if acuan > 0 else 0.0
        self.sta_puncak = max(self.sta_puncak, self.sta)
        self.rasio_puncak = max(self.rasio_puncak, self.rasio)

        if not self.aktif:
            # LTA hanya belajar saat tenang. Deque membuang yang terlama
            # sendiri, jadi jumlahnya dikurangi dulu sebelum menambah.
            if len(self.lta) == self.lta.maxlen:
                self.lta_jumlah -= self.lta[0]
            self.lta.append(self.sta)
            self.lta_jumlah += self.sta

            if self.rasio <= self.rasio_picu:
                return
            if self.sta < self.sta_minimum:
                self.tertahan += 1
                return
            self._mulai(t_wall)
        else:
            if self.rasio >= self.rasio_lepas:
                self.diam_blok = 0
                self.blok_ramai += 1
            else:
                self.diam_blok += 1

            selesai = self.diam_blok >= self.hold_off_blok
            kepanjangan = len(self.sampel) >= self.maks_n
            if selesai or kepanjangan:
                self._tutup(kepanjangan)

    def _mulai(self, t_wall):
        self.aktif = True
        self.sampel = list(self.pre)
        self.diam_blok = 0
        self.blok_ramai = 1
        self.t_mulai = t_wall - len(self.sampel) / self.fs

        # Baseline event diambil MUNDUR, dari bagian paling awal penyangga
        # pra-trigger — bagian yang masih tenang, sebelum gerakan dimulai.
        # Median, bukan rata-rata, supaya satu ketukan nyasar di detik itu
        # tidak menggesernya.
        #
        # Diambil dari yang paling awal, bukan yang paling tenang sepanjang
        # rekaman: kalau rel hanyut, baseline lama memberi puncak yang salah
        # sebesar hanyutnya. Yang dibutuhkan adalah tenang DAN baru.
        kuota = min(len(self.sampel), int(self.fs * 1.0))
        if kuota:
            awal = sorted(self.sampel[:kuota])
            self.base_saat_trigger = awal[kuota // 2]
        else:
            self.base_saat_trigger = self.baseline.nilai

    def _tutup(self, kepanjangan):
        self.aktif = False

        if kepanjangan:
            # Kereta yang benar-benar lewat tidak menyentuh batas durasi.
            # Yang menyentuhnya berarti sesuatu bergetar terus-menerus —
            # mesin, angin kencang, atau sensor yang bermasalah. LTA
            # dikosongkan supaya ia belajar ulang seperti apa "biasanya"
            # di keadaan yang baru itu.
            self.timeout_beruntun += 1
            self.lta.clear()
            self.lta_jumlah = 0.0

            # Timeout PERTAMA tetap diteruskan: bisa saja itu rangkaian
            # barang yang benar-benar lebih panjang dari batas durasi, dan
            # membuangnya berarti kehilangan data asli. Yang kedua dan
            # seterusnya baru dianggap kemacetan.
            if self.timeout_beruntun >= 2:
                self.sampel = []
                self.pre.clear()
                self.blok_ramai = 0
                print(f"  ! Event ke-{self.timeout_beruntun} berturut-turut "
                      f"ditutup karena kepanjangan ({self.cfg['maks_durasi_detik']} s).\n"
                      f"    Ada yang bergetar terus-menerus di dekat sensor.\n"
                      f"    Jalankan:  python3 strain_agent.py --ukur 60\n"
                      f"    Paket ini TIDAK dikirim ke web.", flush=True)
                return
        else:
            self.timeout_beruntun = 0
        data = self.sampel
        self.sampel = []
        self.pre.clear()

        # Buang ekor diam, sisakan sedikit untuk konteks
        sisa = max(0, self.diam_blok * self.blok_n - int(0.3 * self.fs))
        if sisa > 0:
            data = data[:-sisa]

        puncak = max(abs(v - self.base_saat_trigger) for v in data) if data else 0.0
        dangkal = (self.min_puncak_ue > 0
                   and ke_microstrain(puncak, self.cfg) < self.min_puncak_ue)

        if len(data) < self.min_n or self.blok_ramai < self.min_blok_ramai or dangkal:
            # Bukan kereta: gangguan listrik, getaran sesaat, atau kendaraan
            # yang lewat di dekat rel. Dicatat jumlahnya supaya kalau
            # penolakan jadi ratusan per jam, kelihatan bahwa lantai
            # absolutnya masih terlalu rendah untuk lokasi ini.
            self.ditolak += 1
            self.puncak_tolak = max(self.puncak_tolak, puncak)
            if self.ditolak % 20 == 0:
                print(f"  (ditolak: {self.ditolak} kali, puncak tertinggi "
                      f"{ke_microstrain(self.puncak_tolak, self.cfg):.1f} µε)",
                      flush=True)
            self.blok_ramai = 0
            return

        self.blok_ramai = 0
        self.jumlah_event += 1
        self.saat_selesai(data, self.base_saat_trigger, self.t_mulai, kepanjangan)


# ======================================================================
# Fitur event
# ======================================================================

def cari_puncak(d, ambang, jarak_min):
    """
    Mencari lembah dengan cara daerah, bukan cara titik.

    Satu gandar menghasilkan satu daerah yang nilainya turun di bawah
    ambang, lalu naik lagi. Di dalam daerah itu, derau membuat banyak
    titik minimum lokal — kalau tiap titik dihitung, satu gandar bisa
    terbaca sebagai tiga atau empat. Jadi yang dicari adalah daerahnya,
    lalu diambil satu titik terdalam per daerah.

    Daerah yang terpisah kurang dari jarak_min digabung, supaya satu
    lembah yang sempat naik sekejap tidak terpecah dua.

    Tanpa scipy — numpy belum tentu ada di Pi yang baru dipasang.
    """
    daerah = []
    awal = None
    for i, v in enumerate(d):
        if v < -ambang:
            if awal is None:
                awal = i
        elif awal is not None:
            daerah.append((awal, i))
            awal = None
    if awal is not None:
        daerah.append((awal, len(d)))

    if not daerah:
        return []

    gabung = [list(daerah[0])]
    for a, b in daerah[1:]:
        if a - gabung[-1][1] < jarak_min:
            gabung[-1][1] = b
        else:
            gabung.append([a, b])

    return [min(range(a, b), key=lambda k: d[k]) for a, b in gabung]


def pisah_gandar(d, a, b, prominence):
    """
    Memisahkan gandar-gandar di dalam satu kelompok (bogie).

    Dua gandar dalam satu bogie terpisah sekitar 1,8 meter, dan di antara
    keduanya sinyal hanya naik sebagian — tidak pernah kembali ke nol.
    Jadi mereka tidak bisa dipisahkan dengan ambang mendatar: ambang yang
    cukup tinggi untuk memisahkan gandar berat akan melewatkan gandar
    ringan sama sekali.

    Yang dipakai di sini ukuran naik-turun setempat (prominence): dua
    lembah dianggap terpisah kalau sinyal di antaranya naik lebih dari
    sekian. Ukuran ini tidak bergantung pada kedalaman mutlak, jadi
    gandar berat dan ringan diperlakukan sama.
    """
    # Kumpulkan titik ekstrem bergantian: lembah, bukit, lembah, ...
    ext = []
    for i in range(a + 1, b - 1):
        if d[i] <= d[i - 1] and d[i] < d[i + 1]:
            ext.append(("min", i))
        elif d[i] >= d[i - 1] and d[i] > d[i + 1]:
            ext.append(("max", i))
    if not ext:
        return [min(range(a, b), key=lambda k: d[k])]

    # Gabungkan lembah yang dipisahkan bukit terlalu rendah
    lembah = [i for t, i in ext if t == "min"]
    if len(lembah) <= 1:
        return lembah or [min(range(a, b), key=lambda k: d[k])]

    hasil = [lembah[0]]
    for j in lembah[1:]:
        p = hasil[-1]
        bukit = max(d[p:j + 1])
        naik = min(bukit - d[p], bukit - d[j])
        if naik >= prominence:
            hasil.append(j)
        elif d[j] < d[p]:
            hasil[-1] = j
    return hasil


def hitung_fitur(data, baseline, fs, jarak_gandar_m=0.0, koreksi=1.0):
    d = [v - baseline for v in data]
    puncak_nilai = min(d)
    tertinggi = max(d)
    dalam = abs(puncak_nilai)

    # Lapis 1 — kelompok. Ambang 40% memisahkan bogie dengan andal:
    # di antara dua bogie sinyal kembali mendekati baseline.
    jarak_min = int(0.04 * fs)
    kelompok = []
    awal = None
    for i, v in enumerate(d):
        if v < -dalam * 0.4:
            if awal is None:
                awal = i
        elif awal is not None:
            kelompok.append((awal, i))
            awal = None
    if awal is not None:
        kelompok.append((awal, len(d)))

    gabung = []
    for a, b in kelompok:
        if gabung and a - gabung[-1][1] < jarak_min:
            gabung[-1][1] = b
        else:
            gabung.append([a, b])

    # Lapis 2 — gandar di dalam tiap kelompok
    prominence = dalam * 0.10
    gandar = []
    for a, b in gabung:
        gandar.extend(pisah_gandar(d, a, b, prominence))
    gandar.sort()

    fitur = {
        "n_sampel": len(data),
        "durasi_s": round(len(data) / fs, 3),
        "baseline_hitungan": round(baseline, 1),
        "puncak_hitungan": round(puncak_nilai, 1),
        "rentang_hitungan": round(tertinggi - puncak_nilai, 1),
        "jumlah_kelompok": len(gabung),
        "jumlah_gandar": len(gandar),
    }

    # Jarak antar gandar DALAM satu kelompok saja. Jarak antar kelompok
    # berganti-ganti (dalam gerbong vs antar gerbong), jadi mediannya
    # tidak berarti apa-apa.
    dalam_kelompok = []
    for a, b in gabung:
        g = [i for i in gandar if a <= i < b]
        dalam_kelompok += [(g[i + 1] - g[i]) / fs for i in range(len(g) - 1)]

    if dalam_kelompok:
        dalam_kelompok.sort()
        dt = dalam_kelompok[len(dalam_kelompok) // 2]
        fitur["jarak_gandar_s"] = round(dt, 4)
        # Kecepatan hanya dihitung kalau jarak gandar yang sebenarnya
        # sudah diketahui dari data sarana KAI. Lebih baik kosong
        # daripada diisi angka yang berdasar asumsi diam-diam.
        if jarak_gandar_m > 0 and 0.02 < dt < 2.0:
            fitur["kecepatan_kmh"] = round(jarak_gandar_m / dt * 3.6 * koreksi, 1)
    return fitur


def ke_microstrain(hitungan, cfg):
    volt = hitungan * (cfg["vref"] / cfg["gain"]) / 8388608.0
    return volt / cfg["v_excitation"] / (cfg["gauge_factor"] * cfg["bridge_n"] / 4.0) * 1e6


# ======================================================================
# Antrean pengiriman
# ======================================================================

class Pengirim(threading.Thread):
    """
    Event ditulis dulu ke folder antrean, baru dikirim. Kalau jaringan
    putus, berkasnya tetap di sana dan dicoba lagi nanti — inilah
    keuntungan utama punya Raspberry Pi di tengah dibanding ESP32 yang
    mengirim langsung ke web.
    """

    def __init__(self, cfg, dir_antrean):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.dir = dir_antrean
        self.berhenti = threading.Event()
        self.terkirim = 0
        self.gagal = 0
        os.makedirs(self.dir, exist_ok=True)

    def run(self):
        while not self.berhenti.is_set():
            try:
                antre = sorted(f for f in os.listdir(self.dir) if f.endswith(".json"))
            except OSError:
                antre = []

            for nama in antre:
                if self.berhenti.is_set():
                    break
                path = os.path.join(self.dir, nama)
                try:
                    with open(path, "rb") as f:
                        body = f.read()
                    if self._kirim(body):
                        os.remove(path)
                        self.terkirim += 1
                    else:
                        self.gagal += 1
                        break        # jaringan sedang bermasalah, tunggu
                except OSError:
                    break

            self.berhenti.wait(20)

    def _kirim(self, body):
        req = urllib.request.Request(
            self.cfg["url_event"], data=body, method="POST",
            headers={"Content-Type": "application/json",
                     "X-Api-Key": self.cfg.get("api_key", "")})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return 200 <= r.status < 300
        except urllib.error.HTTPError as e:
            # 4xx berarti paketnya yang salah, bukan jaringannya. Dibuang
            # supaya satu paket cacat tidak menyumbat antrean selamanya.
            if 400 <= e.code < 500:
                print(f"  paket ditolak server ({e.code}) — dibuang dari antrean")
                return True
            return False
        except Exception:
            return False


# ======================================================================
# Penulis arsip mentah
# ======================================================================

class ArsipMentah:
    def __init__(self, root, fs, aktif=True):
        self.root, self.fs, self.aktif = root, fs, aktif
        self.key = self.f = self.path = None
        self.n = 0

    def tulis(self, blob, nsamp):
        if not self.aktif:
            return
        now = datetime.now()
        key = now.strftime("%Y-%m-%d_%H")
        if key != self.key:
            self.tutup()
            d = os.path.join(self.root, now.strftime("%Y-%m-%d"))
            os.makedirs(d, exist_ok=True)
            self.path = os.path.join(d, f"raw_{key}.i32")
            self.f = open(self.path, "ab", buffering=1 << 16)
            self.key, self.n = key, 0
        self.f.write(blob)
        self.n += nsamp

    def tutup(self):
        if self.f:
            self.f.close()
            self.f = None


# ======================================================================
# Pembersihan berkala
# ======================================================================

def bersihkan(cfg, dir_data, diam=False):
    """
    Membuang berkas lama. Dihapus, bukan dipindah atau dipadatkan.

    Arsip mentah tumbuh sekitar 345 MB per hari pada 1000 Hz. Tanpa
    pembersihan, kartu 32 GB penuh dalam hitungan minggu, dan saat penuh
    agent berhenti menulis tanpa ada yang tahu di lapangan.

    Folder antrean TIDAK PERNAH disentuh. Isinya event yang belum sampai
    ke web; menghapusnya berarti menghilangkan data yang belum ada
    salinannya di mana pun.
    """
    hari_mentah = int(cfg.get("simpan_mentah_hari", 14))
    hari_event  = int(cfg.get("simpan_event_hari", 90))
    sekarang = time.time()
    terhapus, dibebaskan = 0, 0

    def buang(path):
        nonlocal terhapus, dibebaskan
        try:
            ukuran = os.path.getsize(path)
            os.remove(path)
            terhapus += 1
            dibebaskan += ukuran
        except OSError:
            pass

    # --- Arsip mentah: folder per tanggal ---
    if hari_mentah > 0 and os.path.isdir(dir_data):
        batas = sekarang - hari_mentah * 86400
        for nama in os.listdir(dir_data):
            # Hanya folder bernama tanggal. 'events' dan 'antrean' lolos
            # dari pola ini dengan sendirinya.
            if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", nama):
                continue
            d = os.path.join(dir_data, nama)
            if not os.path.isdir(d):
                continue
            try:
                umur = datetime.strptime(nama, "%Y-%m-%d").timestamp()
            except ValueError:
                continue
            if umur >= batas:
                continue
            for berkas in os.listdir(d):
                buang(os.path.join(d, berkas))
            try:
                os.rmdir(d)
            except OSError:
                pass

    # --- Salinan event lokal ---
    dir_event = os.path.join(dir_data, "events")
    if hari_event > 0 and os.path.isdir(dir_event):
        batas = sekarang - hari_event * 86400
        for nama in os.listdir(dir_event):
            if not nama.endswith(".json"):
                continue
            path = os.path.join(dir_event, nama)
            try:
                if os.path.getmtime(path) < batas:
                    buang(path)
            except OSError:
                pass

    if terhapus and not diam:
        print(f"  Pembersihan: {terhapus} berkas dihapus, "
              f"{dibebaskan / 1048576:.0f} MB dibebaskan "
              f"(mentah >{hari_mentah} hari, event >{hari_event} hari)",
              flush=True)
    return terhapus, dibebaskan


def sisa_ruang_persen(path):
    """Persentase ruang terpakai pada partisi tempat path berada."""
    try:
        st = os.statvfs(path)
        total = st.f_blocks * st.f_frsize
        bebas = st.f_bavail * st.f_frsize
        return 100.0 * (total - bebas) / total if total else 0.0
    except OSError:
        return 0.0



# ======================================================================
# Pembukaan port yang tahan lepas-pasang
# ======================================================================

def cari_port(port_diminta):
    """
    Mencari port serial yang benar-benar ada.

    Setelah ESP32 lepas dan tersambung lagi — karena kabel goyang, daya
    turun sesaat, atau papan me-reset dirinya — Linux sering memberinya
    nomor baru: ttyUSB0 jadi ttyUSB1. Kalau yang dicari nomor lama,
    programnya gagal terus padahal alatnya sudah normal.

    Jadi: pakai yang diminta kalau ada; kalau tidak, cari sendiri.
    """
    if port_diminta and os.path.exists(port_diminta):
        return port_diminta

    # Jalur by-id menempel pada perangkatnya, bukan urutan colok
    import glob
    kandidat = sorted(glob.glob('/dev/serial/by-id/*'))
    if kandidat:
        return kandidat[0]

    kandidat = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    return kandidat[0] if kandidat else None


def buka_port(cfg, diam=False):
    """
    Membuka port, menunggu sampai berhasil. Tidak pernah menyerah:
    di lapangan tidak ada yang bisa menyalakannya kembali secara manual.
    """
    jeda = 2
    sudah_lapor = False
    while True:
        port = cari_port(cfg["port"])
        if port:
            try:
                ser = serial.Serial(port, cfg["baud"], timeout=1)
                if port != cfg["port"]:
                    print(f"  Port berpindah ke {port} (site.json menyebut {cfg['port']})")
                elif sudah_lapor:
                    print(f"  Tersambung lagi ke {port}")
                return ser
            except serial.SerialException as e:
                if not sudah_lapor and not diam:
                    print(f"  Gagal membuka {port}: {e}")
        elif not sudah_lapor and not diam:
            print(f"  Menunggu ESP32 tercolok ({cfg['port']} belum ada)...")

        sudah_lapor = True
        time.sleep(jeda)
        jeda = min(jeda * 1.5, 30)      # mundur bertahap, maksimum 30 detik


# ======================================================================
# Program utama
# ======================================================================

def ukur_derau(cfg, detik):
    """
    Mengukur lantai derau di lokasi, lalu mengusulkan ambang.

    Ambang 9.300 di berkas contoh berasal dari uji meja: sensor tenang di
    ruangan tenang. Di rel, dasarnya berbeda — getaran tanah dari jalan,
    induksi dari kabel sinyal, dan pemuaian termal rel itu sendiri. Ambang
    yang lebih rendah daripada derau membuat deteksi menyala terus.

    Jalankan saat TIDAK ada kereta lewat.
    """
    print(f"Mengukur derau selama {detik} detik. Pastikan tidak ada kereta "
          f"lewat selama pengukuran.\n")
    ser = buka_port(cfg)
    fs = cfg["fs_hz"]
    base = Baseline(fs, cfg["baseline_detik"])

    buf = bytearray()
    simpangan, level = [], []
    t0 = time.time()

    while time.time() - t0 < detik:
        try:
            chunk = ser.read(4096)
        except (serial.SerialException, OSError) as e:
            print(f"  Port terputus: {e}")
            break
        if chunk:
            buf.extend(chunk)

        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if len(buf) > 2:
                    del buf[:-1]
                break
            if i > 0:
                del buf[:i]
            if len(buf) < HEADER:
                break
            if buf[2] != VERSION or not (0 < buf[3] <= 200):
                del buf[:2]
                continue
            n = buf[3]
            total = HEADER + 4 * n + 2
            if len(buf) < total:
                break
            frame = bytes(buf[:total])
            del buf[:total]
            if struct.unpack_from("<H", frame, total - 2)[0] != crc16(frame[2:total - 2]):
                continue
            for v in struct.unpack(f"<{n}i", frame[HEADER:HEADER + 4 * n]):
                base.tambah(v)
                level.append(v)
                if base.siap:
                    simpangan.append(abs(v - base.nilai))

        sisa = detik - (time.time() - t0)
        if int(sisa) % 10 == 0 and sisa > 1:
            print(f"  sisa {int(sisa)} detik, {len(level):,} sampel"
                  .replace(",", "."), end="\r", flush=True)

    ser.close()

    if len(simpangan) < fs * 5:
        print("\nData terlalu sedikit. Periksa sambungan ESP32 lalu ulangi.")
        return

    simpangan.sort()
    p = lambda q: simpangan[min(len(simpangan) - 1, int(len(simpangan) * q))]
    hanyut = max(level[-fs:]) - min(level[:fs]) if len(level) > 2 * fs else 0

    # Ambang diambil dari nilai terbesar yang sempat muncul saat tenang,
    # bukan dari rms. Yang memicu deteksi palsu memang puncak derau, bukan
    # rata-ratanya. Margin 30% untuk derau yang belum sempat terlihat.
    usul = int(round(max(simpangan) * 1.3 / 100.0) * 100)

    # Lantai STA: sekitar 3x keramaian khas saat diam. Dihitung dari blok
    # 50 ms, ukuran yang sama persis dipakai detektor — bukan dari
    # simpangan terhadap baseline, yang ikut terbawa hanyut.
    blok = max(1, int(fs * float(cfg.get("sta_detik", 0.05))))
    stas = []
    for i in range(0, max(0, len(level) - blok), blok):
        q = level[i:i + blok]
        mq = sum(q) / blok
        stas.append((sum((v - mq) ** 2 for v in q) / blok) ** 0.5)
    stas.sort()
    sta_khas = stas[len(stas) // 2] if stas else 0.0
    usul_lantai = max(100, int(round(sta_khas * 3 / 100.0) * 100))

    print(f"\n\n=== Hasil pengukuran, {len(simpangan):,} sampel ===".replace(",", "."))
    print(f"  simpangan median    : {p(0.50):>10,.0f} hitungan".replace(",", "."))
    print(f"  simpangan p99       : {p(0.99):>10,.0f}".replace(",", "."))
    print(f"  simpangan p99,9     : {p(0.999):>10,.0f}".replace(",", "."))
    print(f"  simpangan tertinggi : {max(simpangan):>10,.0f}".replace(",", "."))
    print(f"  hanyut level        : {hanyut:>10,.0f} selama {detik} detik"
          .replace(",", "."))
    print(f"  STA khas (50 ms)    : {sta_khas:>10,.0f}".replace(",", "."))
    print()
    print(f"  Lantai sekarang : {float(cfg.get('sta_minimum', 3000)):>10,.0f} hitungan"
          f"  (~{ke_microstrain(float(cfg.get('sta_minimum', 3000)), cfg):.0f} µε)"
          .replace(",", "."))
    print(f"  USULAN          : {usul_lantai:>10,.0f} hitungan"
          f"  (~{ke_microstrain(usul_lantai, cfg):.0f} µε)".replace(",", "."))
    print()

    if usul_lantai > float(cfg.get("sta_minimum", 3000)) * 3:
        print("  Deraunya jauh di atas lantai sekarang. Sebelum sekadar")
        print("  menaikkannya, periksa dulu:")
        print("    - perekat gauge sudah benar-benar kering dan menempel?")
        print("    - kabel gauge terpilin dan terlindung (shielded)?")
        print("    - grounding ESP32 dan ADS1220 menyatu?")
        print("    - gain 64 mungkin terlalu tinggi; coba 32.")
        print("  Lantai tinggi menyembunyikan derau, tidak menghilangkannya —")
        print("  dan ikut menyembunyikan kereta yang ringan.")
        print()
    print(f'  Kalau sudah yakin, ubah di site.json:  "sta_minimum": {usul_lantai},')
    print()
    print("  Rasio picu dan lepas tidak perlu diubah. Keduanya tidak")
    print("  bersatuan, jadi tetap berlaku di lokasi dan bahan mana pun.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="site.json")
    ap.add_argument("--port", default="")
    ap.add_argument("--ambang", type=int, default=0, help="ambang hitungan ADC")
    ap.add_argument("--no-upload", action="store_true", help="jangan kirim ke web")
    ap.add_argument("--no-raw", action="store_true", help="jangan arsipkan data mentah")
    ap.add_argument("--ukur", type=int, default=0, metavar="DETIK",
                    help="ukur lantai derau lalu usulkan ambang, tanpa merekam")
    args = ap.parse_args()

    cfg = muat_konfigurasi(args.config)
    if args.port:
        cfg["port"] = args.port
    if args.ambang:
        cfg["ambang_hitungan"] = args.ambang
    if args.no_raw:
        cfg["arsip_mentah"] = False

    if args.ukur:
        ukur_derau(cfg, args.ukur)
        return

    fs = cfg["fs_hz"]
    dir_event = os.path.join(cfg["dir_data"], "events")
    dir_antrean = os.path.join(cfg["dir_data"], "antrean")
    os.makedirs(dir_event, exist_ok=True)

    print(f"Unit   : {cfg['unit_id']}  (deployment {cfg['deployment_id']})")
    print(f"Lokasi : {cfg['lokasi']}")
    print(f"Deteksi: STA/LTA — rasio keramaian, bukan ambang posisi")
    print(f"         STA {float(cfg.get('sta_detik', 0.05))*1000:.0f} ms, "
          f"LTA {float(cfg.get('lta_detik', 30)):.0f} s, "
          f"picu {float(cfg.get('rasio_picu', 4.0)):.1f}x, "
          f"lepas {float(cfg.get('rasio_lepas', 2.0)):.1f}x")
    print(f"         puncak minimum "
          f"{float(cfg.get('min_puncak_ue', 0)):.0f} µε"
          + ("  (MATI — event dangkal akan ikut terkirim)"
             if not float(cfg.get("min_puncak_ue", 0)) else ""))
    print(f"         lantai {float(cfg.get('sta_minimum', 3000)):,.0f} hitungan"
          f"  (~{ke_microstrain(float(cfg.get('sta_minimum', 3000)), cfg):.1f} µε)"
          .replace(",", "."))
    print(f"Port   : {cfg['port']} @ {cfg['baud']}\n")

    ser = buka_port(cfg)

    arsip = ArsipMentah(cfg["dir_data"], fs, cfg["arsip_mentah"])
    pengirim = None
    if not args.no_upload:
        pengirim = Pengirim(cfg, dir_antrean)
        pengirim.start()

    stat = {"event": 0, "bad_crc": 0, "lost": 0, "sampel": 0, "putus": 0}

    def simpan_event(data, baseline, t_mulai, kepanjangan):
        fitur = hitung_fitur(data, baseline, fs,
                             cfg.get("jarak_gandar_m", 0.0),
                             cfg.get("koreksi_kecepatan", 1.0))
        waktu = datetime.fromtimestamp(t_mulai)
        stamp = waktu.strftime("%Y%m%d_%H%M%S_%f")[:-3]

        # Waveform disimpan sebagai selisih dari baseline, dalam int16
        # berskala — bukan int32.
        #
        # Skala dipilih per event supaya nilai terbesarnya pas di int16.
        # Kehilangan resolusinya beberapa hitungan ADC saja, jauh di bawah
        # derau yang sekitar 1.500, jadi tidak ada informasi yang hilang.
        # Yang didapat: ukuran kiriman tinggal separuh — berarti sekitar
        # 1,4 MB per hari, bukan 2,8 MB. Penting di lokasi yang hanya
        # punya sinyal seluler.
        rel = [v - baseline for v in data]
        puncak_abs = max(abs(min(rel)), abs(max(rel)), 1.0)
        skala = max(1.0, puncak_abs / 32000.0)

        ringkas = float(cfg.get("ringkas_detik", 0) or 0)
        if ringkas > 0:
            # Peringkasan selubung (envelope), bukan pencuplikan.
            #
            # Mengambil satu sampel tiap detik akan hampir selalu jatuh di
            # sela antar-gandar, karena puncaknya cuma sekitar 6% dari waktu.
            # Puncak strain-nya — satu-satunya angka yang penting untuk
            # kelelahan logam — kemungkinan besar terlewat.
            #
            # Jadi tiap kantong waktu diwakili dua angka: nilai terendah dan
            # tertinggi di dalamnya. Puncaknya terjaga persis, dan grafiknya
            # terbaca sebagai pita yang melebar saat kereta lewat.
            n = max(1, int(fs * ringkas))
            pasang = []
            for i in range(0, len(rel), n):
                potong = rel[i:i + n]
                pasang.append(int(round(min(potong) / skala)))
                pasang.append(int(round(max(potong) / skala)))
            mentah = struct.pack(f"<{len(pasang)}h", *pasang)
            dtype = "int16_minmax"
            kantong_ms = int(ringkas * 1000)
            n_titik = len(pasang) // 2
        else:
            kecil = [int(round(v / skala)) for v in rel]
            mentah = struct.pack(f"<{len(kecil)}h", *kecil)
            dtype = "int16"
            kantong_ms = 0
            n_titik = len(kecil)

        terkompresi = base64.b64encode(zlib.compress(mentah, 6)).decode()

        paket = {
            "unit_id": cfg["unit_id"],
            "deployment_id": cfg["deployment_id"],
            "mulai": waktu.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "fs_hz": fs,
            "pre_trigger_ms": int(cfg["pre_detik"] * 1000),
            "gain": cfg["gain"],
            "bridge_n": cfg["bridge_n"],
            "vref": cfg["vref"],
            "v_excitation": cfg["v_excitation"],
            "gauge_factor": cfg["gauge_factor"],
            "terpotong": bool(kepanjangan),
            "wf_dtype": dtype,
            "wf_skala": round(skala, 6),
            "wf_kantong_ms": kantong_ms,
            "wf_titik": n_titik,
            "waveform_b64": terkompresi,
        }
        paket.update(fitur)
        paket["puncak_ue"] = round(ke_microstrain(fitur["puncak_hitungan"], cfg), 2)
        paket["rentang_ue"] = round(ke_microstrain(fitur["rentang_hitungan"], cfg), 2)

        # Satu salinan untuk arsip lokal, satu untuk antrean kirim.
        with open(os.path.join(dir_event, f"ev_{stamp}.json"), "w") as f:
            json.dump(paket, f)
        if pengirim:
            tmp = os.path.join(dir_antrean, f"ev_{stamp}.tmp")
            with open(tmp, "w") as f:
                json.dump(paket, f)
            os.rename(tmp, tmp[:-4] + ".json")   # atomik: antrean tak pernah baca separuh

        stat["event"] += 1
        tanda = " [TERPOTONG]" if kepanjangan else ""
        print(f"  ▸ EVENT {stat['event']:3d}  {waktu.strftime('%H:%M:%S')}  "
              f"puncak {paket['puncak_ue']:8.1f} µε  "
              f"durasi {fitur['durasi_s']:5.2f} s  "
              f"gandar {fitur['jumlah_gandar']:3d}  "
              f"{len(terkompresi)/1024:.1f} KB"
              + (f"  {fitur['kecepatan_kmh']:5.1f} km/h" if "kecepatan_kmh" in fitur else "")
              + tanda)

    detektor = Detektor(cfg, simpan_event)

    jalan = {"on": True}
    signal.signal(signal.SIGINT, lambda *_: jalan.update(on=False))
    signal.signal(signal.SIGTERM, lambda *_: jalan.update(on=False))

    buf = bytearray()
    prev_seq = None
    t_mulai_program = time.time()
    t_lapor = t_mulai_program
    t_bersih = t_mulai_program
    jendela = []

    # Sekali saat mulai: kalau Pi sempat mati berminggu-minggu, berkas lama
    # sudah menumpuk sebelum baris pertama ditulis.
    bersihkan(cfg, cfg["dir_data"])

    print("Berjalan. Ctrl-C untuk berhenti.\n")
    print(f"{'waktu':>8} {'laju':>9} {'baseline':>11} {'sta':>9} "
          f"{'lta':>9} {'rasio':>7} {'event':>6} {'antre':>6}")

    while jalan["on"]:
        # ESP32 bisa lepas sewaktu-waktu: kabel goyang, daya turun sesaat,
        # atau papan me-reset. Dulu program langsung mati dan systemd
        # menyalakannya ulang — tapi itu menghapus penyangga pra-trigger
        # dan baseline, jadi kereta yang lewat saat itu hilang. Sekarang
        # portnya dibuka ulang di tempat, tanpa program berhenti.
        try:
            chunk = ser.read(4096)
        except (serial.SerialException, OSError) as e:
            print(f"  Port terputus: {e}")
            stat["putus"] += 1
            try:
                ser.close()
            except Exception:
                pass
            buf.clear()
            prev_seq = None          # nomor urut mulai lagi dari awal
            ser = buka_port(cfg)
            continue

        if chunk:
            buf.extend(chunk)

        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if len(buf) > 2:
                    del buf[:-1]
                break
            if i > 0:
                sampah = bytes(buf[:i])
                if sampah.strip().startswith(b"#"):
                    for line in sampah.split(b"\n"):
                        if line.strip():
                            print("  ESP32:", line.decode(errors="replace").strip())
                del buf[:i]

            if len(buf) < HEADER:
                break
            if buf[2] != VERSION or not (0 < buf[3] <= 200):
                del buf[:2]
                continue
            n = buf[3]
            total = HEADER + 4 * n + 2
            if len(buf) < total:
                break

            frame = bytes(buf[:total])
            del buf[:total]

            if struct.unpack_from("<H", frame, total - 2)[0] != crc16(frame[2:total - 2]):
                stat["bad_crc"] += 1
                continue

            seq = struct.unpack_from("<I", frame, 4)[0]
            if prev_seq is not None and seq != prev_seq + 1:
                hilang = (seq - prev_seq - 1) * n
                if 0 < hilang < 10_000_000:
                    stat["lost"] += hilang
            prev_seq = seq

            payload = frame[HEADER:HEADER + 4 * n]
            arsip.tulis(payload, n)

            nilai = struct.unpack(f"<{n}i", payload)
            detektor.olah(nilai, time.time())

            stat["sampel"] += n
            jendela.extend(nilai)

        now = time.time()
        if now - t_bersih >= 3600:
            t_bersih = now
            bersihkan(cfg, cfg["dir_data"])

            # Jaring terakhir. Kalau kartu tetap hampir penuh meski sudah
            # dibersihkan, retensi dipangkas sendiri sampai ada ruang.
            # Arsip mentah yang hilang jauh lebih murah daripada agent yang
            # berhenti menulis di lokasi yang tidak bisa dijangkau hari itu.
            persen = sisa_ruang_persen(cfg["dir_data"])
            hari = int(cfg.get("simpan_mentah_hari", 14))
            while persen >= 90 and hari > 1:
                hari = max(1, hari // 2)
                cfg["simpan_mentah_hari"] = hari
                print(f"  ! Kartu {persen:.0f}% penuh. Retensi arsip mentah "
                      f"dipangkas ke {hari} hari.", flush=True)
                bersihkan(cfg, cfg["dir_data"])
                persen = sisa_ruang_persen(cfg["dir_data"])

        if now - t_lapor >= 2.0:
            dt = now - t_lapor
            t_lapor = now
            if jendela:
                b = detektor.baseline.nilai
                lta = (detektor.lta_jumlah / len(detektor.lta)) if detektor.lta else 0.0
                sta_p, rasio_p = detektor.ambil_puncak()
                try:
                    antre = len([f for f in os.listdir(dir_antrean)
                                 if f.endswith(".json")]) if pengirim else 0
                except OSError:
                    antre = 0
                if not detektor.siap:
                    status = "belajar LTA"
                elif detektor.aktif:
                    status = "AKTIF"
                else:
                    status = ""
                if stat["putus"]:
                    status += f"  putus {stat['putus']}x"
                print(f"{time.strftime('%H:%M:%S'):>8} {len(jendela)/dt:8.1f}/s "
                      f"{b:11.0f} {sta_p:9.0f} {lta:9.0f} "
                      f"{rasio_p:6.1f}x "
                      f"{stat['event']:6d} {antre:6d}  {status}")
            jendela = []

    # --- Penutupan ---
    arsip.tutup()
    ser.close()
    if pengirim:
        pengirim.berhenti.set()
        pengirim.join(timeout=5)

    durasi = time.time() - t_mulai_program
    print(f"\nSelesai. {durasi/60:.1f} menit, {stat['sampel']:,} sampel."
          .replace(",", "."))
    print(f"Event terdeteksi : {stat['event']}")
    if detektor.tertahan:
        print(f"Lonjakan ditahan  : {detektor.tertahan}"
              f"  (lewat ambang tapi tidak bertahan {cfg.get('debounce_detik', 0.008)*1000:.0f} ms)")
    if detektor.ditolak:
        print(f"Derau ditolak    : {detektor.ditolak}"
              f"  (puncak tertinggi {detektor.puncak_tolak:,.0f} hitungan)"
              .replace(",", "."))
        if detektor.ditolak > max(5, stat['event'] * 3):
            print("  Penolakan jauh lebih banyak daripada event. Ambangnya "
                  "kemungkinan masih terlalu rendah untuk lokasi ini —\n"
                  "  naikkan ambang_hitungan sedikit di atas puncak tertinggi "
                  "di baris atas.")
    if pengirim:
        print(f"Terkirim         : {pengirim.terkirim}")
        sisa = len([f for f in os.listdir(dir_antrean) if f.endswith('.json')])
        if sisa:
            print(f"Masih mengantre  : {sisa}  (akan dikirim saat program dijalankan lagi)")
    print(f"Bingkai rusak    : {stat['bad_crc']}   Sampel hilang: {stat['lost']}")
    if stat["putus"]:
        print(f"Port terputus    : {stat['putus']} kali — periksa kabel USB dan catu daya")


if __name__ == "__main__":
    main()
