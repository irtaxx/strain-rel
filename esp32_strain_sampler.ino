/*
 * esp32_strain_sampler — TAHAP 1  (ADS1220)
 * -----------------------------------------------------------------------
 * Tugas ESP32 di sini HANYA satu: membaca ADC pada laju tetap dan
 * mengirim HITUNGAN MENTAH ke Raspberry Pi. Tidak ada konversi ke
 * microstrain, tidak ada kalibrasi nol, tidak ada deteksi kereta.
 *
 * Kenapa mentah?
 *   - Faktor jembatan dan gauge factor pasti akan direvisi setelah
 *     kalibrasi lapangan. Kalau konversinya di sini, data lama tidak bisa
 *     dihitung ulang dan tiap revisi berarti datang ke lokasi.
 *   - Baseline bergeser sepanjang hari mengikuti suhu. Mengunci nol sekali
 *     saat boot justru merusak data, bukan memperbaikinya.
 *   Pi yang mengurus baseline, dan konversi dilakukan saat analisis.
 *
 * ADC   : ADS1220 (24-bit, SPI), 1000 SPS, konversi berkelanjutan
 * Waktu : dari pin DRDY lewat interrupt, bukan polling di loop()
 * -----------------------------------------------------------------------
 */

#include <Arduino.h>
#include <SPI.h>

// ====================== YANG BOLEH DIUBAH ======================

#define SIMULATE_MODE     0      // 1 = data tiruan, tanpa ADS1220
#define DEBUG_TEKS        0      // 1 = cetak angka terbaca (uji di Serial Monitor)

#define SERIAL_BAUD       921600
#define SAMPLES_PER_BATCH 50     // 50 sampel @1000 Hz = kirim tiap 50 ms

// Pin (sesuai rangkaian Anda)
#define PIN_CS    27
#define PIN_DRDY  25             // harus pin yang bisa interrupt
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23

// Gain PGA. Lihat catatan panjang di bawah sebelum mengubah.
#define ADC_GAIN_CODE  GAIN_64

// ===============================================================
/*
 * CATATAN GAIN — bagian terpenting di berkas ini.
 *
 * Kode Anda sebelumnya memakai gain 1 dengan PGA di-bypass (reg0 = 0x01).
 * Dengan begitu seluruh rentang ±2,048 V dipakai, padahal keluaran
 * jembatan strain gauge hanya beberapa milivolt. Derau masukan ADC
 * besarnya kurang lebih tetap dalam satuan volt pada laju cuplik tertentu,
 * jadi memperkuat sinyal SEBELUM didigitalkan menaikkan rasio
 * sinyal-terhadap-derau secara langsung, hampir sebanding dengan gain.
 *
 * Default di sini gain 64 (rentang ±32 mV), bukan 128, karena jembatan
 * yang tidak seimbang sempurna punya offset awal. Gauge 350 Ω dengan
 * selisih 1% pada eksitasi 3,3 V sudah menghasilkan offset ~8 mV — pada
 * gain 128 (rentang ±16 mV) itu sudah setengah jalan menuju mentok.
 *
 * Cara menentukan yang tepat untuk rangkaian Anda:
 *   1. Jalankan dengan gain 64, lihat kolom min/maks di record_raw.py
 *   2. Kalau nilainya jauh dari ±8.388.608 (mentok), naikkan ke 128
 *   3. Kalau justru sering menyentuh batas, turunkan ke 32
 */
#define GAIN_1    0
#define GAIN_2    1
#define GAIN_4    2
#define GAIN_8    3
#define GAIN_16   4
#define GAIN_32   5
#define GAIN_64   6
#define GAIN_128  7

// Perintah ADS1220
#define CMD_RESET   0x06
#define CMD_START   0x08
#define CMD_RDATA   0x10
#define CMD_WREG    0x40
#define CMD_RREG    0x20

// ---- Format bingkai ke Pi (little-endian) ----
//   0  magic    2B   0xA5 0x5A
//   2  version  1B   1
//   3  n        1B   jumlah sampel
//   4  seq      4B   nomor urut bingkai
//   8  t_us     8B   waktu mikro sampel PERTAMA bingkai ini
//  16  samples  n*4B int32 hitungan ADC mentah
//  16+4n crc16  2B   CRC-16/CCITT-FALSE atas byte 2..(16+4n-1)
#define FRAME_MAGIC_0 0xA5
#define FRAME_MAGIC_1 0x5A
#define FRAME_VERSION 1

struct Batch {
  uint32_t seq;
  uint64_t t_us;
  int32_t  s[SAMPLES_PER_BATCH];
};

static SPIClass          adsSpi(VSPI);
static SPISettings       spiCfg(2000000, MSBFIRST, SPI_MODE1);
static QueueHandle_t     qBatch;
static SemaphoreHandle_t semDrdy;

static volatile uint32_t dropped = 0;    // bingkai dibuang: pengirim tertinggal
static volatile uint32_t missed  = 0;    // DRDY datang sebelum sampel sebelumnya dibaca

// ---------------------------------------------------------------
// CRC-16/CCITT-FALSE — Pi memakai ini untuk memastikan bingkai utuh.
// Tanpa pemeriksaan ini, satu byte rusak di kabel USB akan menggeser
// seluruh sisa aliran data tanpa ada yang menyadari.
// ---------------------------------------------------------------
static uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*p++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

#if !SIMULATE_MODE

static inline void csLow()  { digitalWrite(PIN_CS, LOW);  }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

static void adsCmd(uint8_t cmd) {
  adsSpi.beginTransaction(spiCfg);
  csLow();
  adsSpi.transfer(cmd);
  csHigh();
  adsSpi.endTransaction();
}

static void adsWriteReg(uint8_t reg, uint8_t val) {
  adsSpi.beginTransaction(spiCfg);
  csLow();
  adsSpi.transfer(CMD_WREG | ((reg & 0x03) << 2));   // nn=00 -> tulis 1 byte
  adsSpi.transfer(val);
  csHigh();
  adsSpi.endTransaction();
}

static uint8_t adsReadReg(uint8_t reg) {
  adsSpi.beginTransaction(spiCfg);
  csLow();
  adsSpi.transfer(CMD_RREG | ((reg & 0x03) << 2));
  uint8_t v = adsSpi.transfer(0x00);
  csHigh();
  adsSpi.endTransaction();
  return v;
}

static inline int32_t adsReadSample() {
  adsSpi.beginTransaction(spiCfg);
  csLow();
  adsSpi.transfer(CMD_RDATA);
  uint8_t msb = adsSpi.transfer(0x00);
  uint8_t mid = adsSpi.transfer(0x00);
  uint8_t lsb = adsSpi.transfer(0x00);
  csHigh();
  adsSpi.endTransaction();

  int32_t v = ((int32_t)msb << 16) | ((int32_t)mid << 8) | lsb;
  if (v & 0x00800000) v |= 0xFF000000;      // perluasan tanda 24 -> 32 bit
  return v;
}

static void IRAM_ATTR onDrdy() {
  BaseType_t hp = pdFALSE;
  if (xSemaphoreGiveFromISR(semDrdy, &hp) != pdTRUE) missed++;
  if (hp) portYIELD_FROM_ISR();
}

/* Mengembalikan true kalau ADS1220 benar-benar menjawab. */
static bool adsBegin() {
  pinMode(PIN_CS, OUTPUT);
  csHigh();
  pinMode(PIN_DRDY, INPUT_PULLUP);
  adsSpi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  adsCmd(CMD_RESET);
  delay(10);

  // Reg0: MUX=AIN0-AIN1 (0000), GAIN sesuai pilihan, PGA aktif (bit0 = 0)
  const uint8_t reg0 = (0x00 << 4) | ((ADC_GAIN_CODE & 0x07) << 1) | 0x00;
  // Reg1: DR=110 (1000 SPS mode normal), MODE=00, CM=1 (berkelanjutan)
  const uint8_t reg1 = 0xC4;
  // Reg2: referensi internal 2,048 V, tanpa filter, IDAC mati
  const uint8_t reg2 = 0x00;
  // Reg3: perilaku DRDY standar
  const uint8_t reg3 = 0x00;

  adsWriteReg(0, reg0);
  adsWriteReg(1, reg1);
  adsWriteReg(2, reg2);
  adsWriteReg(3, reg3);
  delay(5);

  // Baca balik. Kalau SPI salah kabel atau chip tidak hidup, nilainya
  // tidak akan cocok — dan ini jauh lebih baik daripada diam-diam
  // mengirim angka nol seumur hidup.
  bool ok = (adsReadReg(0) == reg0) && (adsReadReg(1) == reg1) &&
            (adsReadReg(2) == reg2) && (adsReadReg(3) == reg3);

  adsCmd(CMD_START);
  delay(5);
  return ok;
}

#endif  // !SIMULATE_MODE

// ---------------------------------------------------------------
// Task pencuplik — core 1, prioritas tinggi
// ---------------------------------------------------------------
static void taskSampler(void *) {
  Batch b;
  uint32_t seq = 0;
  uint8_t  i   = 0;

#if SIMULATE_MODE
  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();
  uint32_t   n = 0;
#endif

  for (;;) {
#if SIMULATE_MODE
    // Sinyal tiruan: derau + satu "kereta" tiap 45 detik, untuk menguji
    // jalur ESP32 -> Pi sebelum sensornya terpasang.
    vTaskDelayUntil(&last, period);
    float t = (float)n / 1000.0f;
    float v = (float)random(-900, 900);
    float ph = fmodf(t, 45.0f);
    if (ph < 12.0f) {
      for (int k = 0; k < 60; k++) {
        float ta = 0.25f + (k / 4) * 0.80f + (k % 4) * 0.11f;
        float d  = (ph - ta) / 0.055f;
        if (d > -6 && d < 6) v -= 700000.0f * expf(-d * d);
      }
    }
    int32_t s = (int32_t)v;
    n++;
#else
    if (xSemaphoreTake(semDrdy, pdMS_TO_TICKS(500)) != pdTRUE) continue;
    int32_t s = adsReadSample();
#endif

#if DEBUG_TEKS
    Serial.println(s);
    continue;
#endif

    if (i == 0) b.t_us = (uint64_t)esp_timer_get_time();
    b.s[i++] = s;

    if (i >= SAMPLES_PER_BATCH) {
      b.seq = seq++;
      i = 0;
      // Antrean penuh berarti pengirim tertinggal. Lebih baik bingkai ini
      // dibuang dan dihitung daripada pencuplik ikut berhenti — nomor urut
      // yang lompat akan terbaca jelas di sisi Pi.
      if (xQueueSend(qBatch, &b, 0) != pdTRUE) dropped++;
    }
  }
}

// ---------------------------------------------------------------
// Task pengirim — core 0
// ---------------------------------------------------------------
static void taskSender(void *) {
  Batch b;
  static uint8_t buf[16 + 4 * SAMPLES_PER_BATCH + 2];

  for (;;) {
    if (xQueueReceive(qBatch, &b, portMAX_DELAY) != pdTRUE) continue;

    buf[0] = FRAME_MAGIC_0;
    buf[1] = FRAME_MAGIC_1;
    buf[2] = FRAME_VERSION;
    buf[3] = SAMPLES_PER_BATCH;
    memcpy(buf + 4,  &b.seq,  4);
    memcpy(buf + 8,  &b.t_us, 8);
    memcpy(buf + 16, b.s, 4 * SAMPLES_PER_BATCH);

    size_t body = 16 + 4 * SAMPLES_PER_BATCH;
    uint16_t c = crc16(buf + 2, body - 2);
    memcpy(buf + body, &c, 2);

    Serial.write(buf, body + 2);
  }
}

void setup() {
  Serial.setTxBufferSize(4096);      // harus sebelum begin()
  Serial.begin(SERIAL_BAUD);
  delay(300);

  semDrdy = xSemaphoreCreateBinary();
  qBatch  = xQueueCreate(24, sizeof(Batch));

#if SIMULATE_MODE
  Serial.println("# MODE SIMULASI - data tiruan, bukan sensor");
#else
  if (!adsBegin()) {
    // Diawali '#' supaya Pi memperlakukannya sebagai catatan, bukan data.
    Serial.println("# ADS1220 tidak menjawab - periksa SPI, CS, dan catu daya");
  } else {
    Serial.printf("# ADS1220 siap, 1000 SPS, gain %d\n", 1 << ADC_GAIN_CODE);
  }
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), onDrdy, FALLING);
#endif

  xTaskCreatePinnedToCore(taskSampler, "sampler", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(taskSender,  "sender",  4096, NULL,  5, NULL, 0);
}

void loop() {
  // Semua pekerjaan ada di dua task. Di sini cukup laporan berkala
  // kalau ada sampel atau bingkai yang terpaksa hilang.
  static uint32_t lastReport = 0, lastDropped = 0, lastMissed = 0;
  if (millis() - lastReport > 10000) {
    lastReport = millis();
    if (dropped != lastDropped || missed != lastMissed) {
      Serial.printf("# bingkai dibuang: %u, sampel terlewat: %u\n",
                    dropped, missed);
      lastDropped = dropped;
      lastMissed  = missed;
    }
  }
  delay(100);
}
