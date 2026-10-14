# Strain Gauge Rel — Tahap 1: Merekam Data Mentah

Tahap ini **belum** mendeteksi kereta dan **belum** mengirim apa pun ke web.
Tujuannya satu: mengumpulkan data nyata dari lokasi, supaya ambang trigger dan
panjang jendela perekaman nanti ditentukan dari angka lapangan, bukan tebakan.

```
ESP32 ──USB──> Raspberry Pi ──> file .i32 di kartu SD
(baca ADC)      (rekam saja)     (disalin ke laptop untuk diperiksa)
```

---

## Isi paket

| Berkas | Jalan di | Fungsi |
|---|---|---|
| `esp32_strain_sampler/esp32_strain_sampler.ino` | ESP32 | Baca ADC 1000×/detik, kirim ke Pi |
| `pi/record_raw.py` | Raspberry Pi | Terima dan simpan ke file |
| `pi/inspect_raw.py` | Laptop | Periksa hasil, usulkan ambang trigger |

---

## 1. Perangkat keras

### Yang dibutuhkan

- ESP32 Dev Module
- **ADS1256** (modul ADC 24-bit SPI) — bukan HX711
- Raspberry Pi (Pi 4, Pi 3, atau Pi Zero 2 W)
- Kabel USB data ESP32 ke Pi, sependek mungkin
- Kartu SD 64 GB untuk Pi

> **Jangan pakai HX711.** Modul itu paling sering muncul kalau mencari
> "ESP32 strain gauge" dan tutorialnya banyak, tapi kecepatan maksimumnya
> 80 sampel/detik. Puncak regangan saat satu gandar lewat lebarnya sekitar
> 80 milidetik — dengan HX711 Anda hanya dapat 6 titik per gandar dan
> bentuknya tidak akan terbaca.

> **ADC bawaan ESP32 juga tidak layak.** Resolusinya 12 bit dan tidak linier.
> Sinyal jembatan strain gauge itu orde mikrovolt.

### Sambungan ESP32 ke ADS1256

| ADS1256 | ESP32 |
|---|---|
| SCLK | GPIO 18 |
| DOUT (MISO) | GPIO 19 |
| DIN (MOSI) | GPIO 23 |
| CS | GPIO 5 |
| DRDY | GPIO 4 |
| RESET | GPIO 2 |
| VCC | 5 V |
| GND | GND |

Sinyal jembatan masuk ke **AIN0 dan AIN1** (mode diferensial).

### Yang gampang terlewat: filter anti-aliasing

Pasang low-pass RC sederhana sebelum masuk ADC, cutoff **200–300 Hz**.
Misalnya R = 4,7 kΩ dan C = 150 nF memberi cutoff ≈ 225 Hz.

Tanpa filter ini, getaran frekuensi tinggi dari roda dan sambungan rel akan
"terlipat" ke pita rendah dan muncul sebagai riak palsu — persis di rentang
frekuensi yang nanti dipakai untuk menghitung gandar. Hasilnya tetap terlihat
masuk akal di grafik, jadi kesalahan ini tidak akan ketahuan sendiri.

### Listrik

Raspberry Pi menyedot 3–5 watt nonstop. Cek dulu apakah panel surya dan
baterai di lokasi sanggup menanggungnya 24 jam. Kalau pas-pasan, pakai
**Pi Zero 2 W** — jauh lebih hemat dan tenaganya masih lebih dari cukup.

---

## 2. Memasang firmware ESP32

1. Buka `esp32_strain_sampler.ino` di Arduino IDE
2. Tools → Board → **ESP32 Dev Module**
3. Upload

**Belum punya ADS1256?** Ubah satu baris di awal berkas:

```cpp
#define SIMULATE_MODE   1
```

ESP32 akan membangkitkan sinyal tiruan lengkap dengan "kereta lewat" tiap 45
detik. Seluruh jalur ESP32 → Pi → file bisa diuji dulu tanpa menunggu
perangkat kerasnya datang. Jangan lupa kembalikan ke `0` kalau sudah terpasang.

---

## 3. Menyiapkan Raspberry Pi

```bash
sudo apt update
sudo apt install -y python3-pip python3-serial
pip3 install pyserial

# supaya bisa akses port serial tanpa sudo
sudo usermod -a -G dialout $USER
# keluar lalu masuk lagi agar berlaku
```

Cari port ESP32:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Biasanya `/dev/ttyUSB0`. Kalau tidak ada, periksa kabelnya — banyak kabel USB
murah yang hanya menyalurkan listrik tanpa jalur data.

---

## 4. Merekam

```bash
cd pi
python3 record_raw.py --port /dev/ttyUSB0 --dir /home/pi/strain-data
```

Tampilan di layar tiap detik:

```
   waktu      laju       rerata   derau rms          min         maks  rusak  hilang
09:14:02   1000.0/s        -1240         178       -1890         1420      0       0
09:14:03    999.8/s        -1238         181      -42100        -1810      0       0
```

Yang perlu dilihat:

- **laju** harus di sekitar 1000/s. Kalau jauh di bawah, ada yang salah di SPI.
- **derau rms** adalah angka kunci — inilah dasar penentuan ambang nanti.
- **rusak** dan **hilang** idealnya tetap 0. Kalau terus bertambah, biasanya
  kabel USB terlalu panjang atau terganggu secara elektrik.

Berhenti dengan Ctrl-C. Berkas tersimpan per jam:

```
strain-data/2026-10-14/raw_2026-10-14_09.i32     sampel mentah
strain-data/2026-10-14/raw_2026-10-14_09.json    keterangan + catatan jeda
```

Satu jam ≈ 14 MB. Satu hari ≈ 345 MB.

### Jalan terus otomatis

```bash
sudo tee /etc/systemd/system/strain-record.service > /dev/null <<'EOF'
[Unit]
Description=Perekam strain gauge
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/strain-tahap1/pi/record_raw.py \
          --port /dev/ttyUSB0 --dir /home/pi/strain-data
WorkingDirectory=/home/pi/strain-tahap1/pi
Restart=always
RestartSec=10
User=pi

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable --now strain-record
sudo systemctl status strain-record
journalctl -u strain-record -f        # lihat keluarannya
```

---

## 5. Memeriksa hasil

Salin berkas ke laptop, lalu:

```bash
pip3 install numpy matplotlib
python3 inspect_raw.py raw_2026-10-14_09.i32
```

Keluarannya:

```
  Durasi                : 60.0 menit  (3.600.000 sampel)
  Baseline              : -28,9 µε
  Derau saat tenang     : 1,48 µε rms
  Simpangan maksimum    : 437,2 µε
  Rasio sinyal/derau    : 295×

  >> Usulan ambang trigger: 8,9 µε dari baseline
```

Plus satu berkas PNG berisi tiga grafik: seluruh rekaman, jendela di sekitar
simpangan terbesar, dan perbesaran 1,5 detik.

**Lihat grafik ketiga.** Kalau pola gandar sudah terbaca di situ — lembah
berpasangan rapat lalu jeda — perangkat kerasnya beres dan kita bisa lanjut ke
Tahap 2. Kalau yang terlihat cuma gundukan mulus tanpa struktur, berarti ada
yang perlu diperbaiki dulu di penguat atau filternya.

---

## 6. Yang perlu dikumpulkan sebelum lanjut

- [ ] Rekaman minimal **4 jam** di jam sibuk, supaya dapat beberapa lintasan
- [ ] Rekaman **1 jam di malam hari** saat sepi, untuk mengukur derau bersih
- [ ] Catat jam berapa saja kereta lewat menurut pengamatan langsung, untuk
      mencocokkan dengan data

Setelah itu kirimkan keluaran `inspect_raw.py` dan berkas PNG-nya. Dari situ
baru kita tentukan ambang trigger dan susun Tahap 2: deteksi event, perhitungan
fitur, dan antrean pengiriman ke web.

---

## Kalau ada masalah

| Gejala | Kemungkinan sebab |
|---|---|
| `# ADS1256 tidak merespons` | Kabel SPI tertukar, atau DRDY bukan di pin yang bisa interrupt |
| Port serial tidak muncul | Kabel USB hanya untuk mengisi daya, tanpa jalur data |
| Laju jauh di bawah 1000/s | Register DRATE belum 0xA1, atau clock SPI kecepatan |
| Banyak bingkai rusak | Kabel USB terlalu panjang; pakai yang pendek dan berpelindung |
| Nilai mentok di batas atas/bawah | Gain PGA terlalu besar, atau jembatan tidak seimbang |
| Derau sangat besar | Pentanahan buruk, atau kabel sensor tidak berpelindung |
