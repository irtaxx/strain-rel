/*
 * esp32_strain_sampler — TAHAP 1
 * -----------------------------------------------------------------------
 * Tugas ESP32 di sini HANYA satu: membaca ADC pada laju tetap dan
 * mengirim apa adanya ke Raspberry Pi lewat USB. Tidak ada deteksi
 * kereta, tidak ada perhitungan apa pun.
 *
 * Alasannya: kode di Pi bisa diubah lewat SSH dari mana saja, sedangkan
 * mengubah kode ini berarti datang ke lokasi membawa laptop. Jadi file
 * ini dibuat sesederhana mungkin supaya tidak pernah perlu diubah lagi.
 *
 * ADC: ADS1256 (24-bit, SPI) pada 1000 SPS.
 * Pewaktuan diambil dari pin DRDY milik ADS1256, bukan dari timer ESP32 —
 * osilator ADC lebih stabil, dan tiap sampel dijamin satu konversi penuh.
 *
 * Belum punya ADS1256? Set SIMULATE_MODE 1 di bawah. ESP32 akan
 * membangkitkan sinyal tiruan supaya jalur ESP32 -> Pi bisa diuji lebih
 * dulu tanpa menunggu perangkat kerasnya datang.
 *
 * Board: ESP32 Dev Module. Arduino IDE / PlatformIO.
 * -----------------------------------------------------------------------
 */

#include <SPI.h>

// ====================== YANG BOLEH DIUBAH ======================

#define SIMULATE_MODE   0        // 1 = data tiruan, tanpa ADS1256

#define SERIAL_BAUD     921600
#define SAMPLES_PER_BATCH 50     // 50 sampel @1000 Hz = kirim tiap 50 ms

// Pin SPI ke ADS1256
#define PIN_CS     5
#define PIN_DRDY   4             // WAJIB pin yang bisa interrupt
#define PIN_RESET  2
#define PIN_SCK    18
#define PIN_MISO   19
#define PIN_MOSI   23

#define ADC_GAIN   ADS_GAIN_64   // jembatan strain gauge keluarannya kecil
#define ADC_SPI_HZ 1700000       // maks fCLKIN/4; ADS1256 kristal 7,68 MHz

// ===============================================================

// Perintah ADS1256
#define CMD_WAKEUP   0x00
#define CMD_RDATAC   0x03
#define CMD_SDATAC   0x0F
#define CMD_RREG     0x10
#define CMD_WREG     0x50
#define CMD_SELFCAL  0xF0
#define CMD_RESET    0xFE

// Register ADS1256
#define REG_STATUS   0x00
#define REG_MUX      0x01
#define REG_ADCON    0x02
#define REG_DRATE    0x03

#define ADS_GAIN_1   0x00
#define ADS_GAIN_2   0x01
#define ADS_GAIN_4   0x02
#define ADS_GAIN_8   0x03
#define ADS_GAIN_16  0x04
#define ADS_GAIN_32  0x05
#define ADS_GAIN_64  0x06

#define DRATE_1000SPS 0xA1

// ---- Format bingkai ke Pi (little-endian) ----
//   0  magic    2B   0xA5 0x5A
//   2  version  1B   1
//   3  n        1B   jumlah sampel
//   4  seq      4B   nomor urut bingkai
//   8  t_us     8B   micros() saat sampel PERTAMA bingkai ini
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

static QueueHandle_t      qBatch;
static SemaphoreHandle_t  semDrdy;
static SPISettings        spiCfg(ADC_SPI_HZ, MSBFIRST, SPI_MODE1);

static volatile uint32_t  dropped = 0;   // bingkai hilang karena antrean penuh

// ---------------------------------------------------------------
// CRC-16/CCITT-FALSE. Pi memakai ini untuk memastikan bingkai utuh;
// kabel USB panjang di lapangan bisa menyisipkan byte rusak.
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

// ---------------------------------------------------------------
// Driver ADS1256 seperlunya
// ---------------------------------------------------------------
#if !SIMULATE_MODE

static inline void csLow()  { digitalWrite(PIN_CS, LOW);  }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

static void adsWaitDrdy() {
  uint32_t guard = 0;
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (++guard > 4000000UL) return;   // jangan menggantung selamanya
  }
}

static void adsWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(val);
  delayMicroseconds(10);
  csHigh();
  SPI.endTransaction();
}

static uint8_t adsReadReg(uint8_t reg) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(CMD_RREG | reg);
  SPI.transfer(0x00);
  delayMicroseconds(10);                // t6 >= 50 * tCLKIN
  uint8_t v = SPI.transfer(0xFF);
  delayMicroseconds(10);
  csHigh();
  SPI.endTransaction();
  return v;
}

static void adsCmd(uint8_t cmd) {
  SPI.beginTransaction(spiCfg);
  csLow();
  SPI.transfer(cmd);
  delayMicroseconds(10);
  csHigh();
  SPI.endTransaction();
}

static bool adsBegin() {
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DRDY, INPUT);
  csHigh();

  digitalWrite(PIN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_RESET, HIGH);
  delay(50);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  adsCmd(CMD_SDATAC);
  delay(10);

  adsWaitDrdy();
  adsWriteReg(REG_STATUS, 0x02);              // buffer analog aktif
  adsWriteReg(REG_MUX,    0x01);              // diferensial AIN0 - AIN1
  adsWriteReg(REG_ADCON,  ADC_GAIN);          // clock out mati, PGA
  adsWriteReg(REG_DRATE,  DRATE_1000SPS);

  adsCmd(CMD_SELFCAL);
  delay(400);                                 // kalibrasi mandiri butuh waktu
  adsWaitDrdy();

  // Baca balik: kalau SPI salah kabel, nilainya tidak akan cocok.
  bool ok = (adsReadReg(REG_DRATE) == DRATE_1000SPS) &&
            ((adsReadReg(REG_ADCON) & 0x07) == ADC_GAIN);

  adsCmd(CMD_RDATAC);                         // mode baca berkelanjutan
  delay(5);
  return ok;
}

// Dalam mode RDATAC cukup baca 3 byte tiap DRDY turun, tanpa perintah.
static inline int32_t adsReadSample() {
  SPI.beginTransaction(spiCfg);
  csLow();
  delayMicroseconds(7);                       // t6
  uint32_t raw = ((uint32_t)SPI.transfer(0xFF) << 16) |
                 ((uint32_t)SPI.transfer(0xFF) << 8)  |
                  (uint32_t)SPI.transfer(0xFF);
  csHigh();
  SPI.endTransaction();

  if (raw & 0x800000) raw |= 0xFF000000;      // perluasan tanda 24 -> 32 bit
  return (int32_t)raw;
}

static void IRAM_ATTR onDrdy() {
  BaseType_t hp = pdFALSE;
  xSemaphoreGiveFromISR(semDrdy, &hp);
  if (hp) portYIELD_FROM_ISR();
}

#endif  // !SIMULATE_MODE

// ---------------------------------------------------------------
// Task pencuplik — core 1, prioritas tinggi, tidak diganggu apa pun
// ---------------------------------------------------------------
static void taskSampler(void *) {
  Batch b;
  uint32_t seq = 0;
  uint8_t  i   = 0;

#if SIMULATE_MODE
  // Sinyal tiruan: derau + satu "kereta" tiap 45 detik, supaya seluruh
  // jalur sampai halaman web bisa diuji sebelum sensornya terpasang.
  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();
  uint32_t   n    = 0;
#endif

  for (;;) {
#if SIMULATE_MODE
    vTaskDelayUntil(&last, period);
    float t = (float)n / 1000.0f;
    float v = (float)random(-800, 800);                 // derau ~±800 hitungan
    float ph = fmodf(t, 45.0f);
    if (ph < 12.0f) {                                   // 12 detik "kereta lewat"
      for (int k = 0; k < 60; k++) {                    // 60 gandar
        float ta = 0.25f + (k / 4) * 0.80f + (k % 4) * 0.11f;
        float d  = (ph - ta) / 0.055f;
        if (d > -6 && d < 6) v -= 420000.0f * expf(-d * d);
      }
    }
    int32_t s = (int32_t)v;
    n++;
#else
    if (xSemaphoreTake(semDrdy, pdMS_TO_TICKS(500)) != pdTRUE) continue;
    int32_t s = adsReadSample();
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
  Serial.setTxBufferSize(4096);     // harus sebelum begin()
  Serial.begin(SERIAL_BAUD);
  delay(300);

  semDrdy = xSemaphoreCreateBinary();
  qBatch  = xQueueCreate(24, sizeof(Batch));

#if SIMULATE_MODE
  Serial.println("# MODE SIMULASI - data tiruan, bukan sensor");
#else
  if (!adsBegin()) {
    // Pesan ini diawali '#' supaya Pi memperlakukannya sebagai catatan,
    // bukan sebagai bingkai data.
    Serial.println("# ADS1256 tidak merespons - periksa kabel SPI dan DRDY");
  } else {
    Serial.println("# ADS1256 siap, 1000 SPS");
  }
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), onDrdy, FALLING);
#endif

  xTaskCreatePinnedToCore(taskSampler, "sampler", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(taskSender,  "sender",  4096, NULL,  5, NULL, 0);
}

void loop() {
  // Semua pekerjaan ada di dua task. Di sini cukup laporan singkat
  // kalau ada bingkai yang terpaksa dibuang.
  static uint32_t lastReport = 0, lastDropped = 0;
  if (millis() - lastReport > 10000) {
    lastReport = millis();
    if (dropped != lastDropped) {
      Serial.printf("# bingkai dibuang: %u\n", dropped);
      lastDropped = dropped;
    }
  }
  delay(100);
}
