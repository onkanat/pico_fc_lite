# Pico FC Lite - Raspberry Pi Pico Flight Controller

Raspberry Pi Pico mikrodenetleyici tabanlı hafif uçuş kontrol sistemi. Bu proje, RC kumanda girişi, sensör entegrasyonu, PID kontrol algoritmaları ve MAVLink telemetri iletişimi için modüler bir yaklaşım benimser.

## 🔧 Donanım Konfigürasyonu

### Raspberry Pi Pico Pinout Referansı

![Raspberry Pi Pico Pinout](docs/pico_pinout.pdf)

> **Not:** Raspberry Pi Pico pin konfigürasyonu ve GPIO atamaları için referans diagramı

### Hızlı Pin Referansı

| GPIO | Fonksiyon | Açıklama | Protokol |
|------|-----------|----------|----------|
| 0 | UART0 TX | MAVLink telemetri çıkışı | UART, 57600 baud |
| 1 | UART0 RX | MAVLink telemetri girişi | UART, 57600 baud |
| 2 | SPI0 SCK | Gelecek: harici flash | SPI |
| 3 | SPI0 MOSI | Gelecek: harici flash | SPI |
| 4 | I2C0 SDA | Sensör veriyolu | I2C, 400kHz |
| 5 | I2C0 SCL | Sensör saat hattı | I2C, 400kHz |
| 6 | Buzzer | Ses uyarıları | PWM |
| 7 | Emergency | Acil durum switch | Digital Input |
| 8 | UART1 TX | GPS komut çıkışı | UART, 9600 baud |
| 9 | UART1 RX | GPS veri girişi | UART, 9600 baud |
| 10 | SPI0 CS | Chip select pin | SPI |
| 11 | Reserved | Gelecek kullanım | - |
| 12 | Reserved | Gelecek kullanım | - |
| 13 | SBUS RX | RC kumanda girişi | PIO, 100kbaud |
| 14 | Armed LED | Arm durumu LED | Digital Output |
| 15 | GPS PPS | Precision timing | Digital Input |
| 16 | Motor 1 | Ön sol motor PWM | PWM, 50Hz |
| 17 | Motor 2 | Ön sağ motor PWM | PWM, 50Hz |
| 18 | Motor 3 | Arka sol motor PWM | PWM, 50Hz |
| 19 | Motor 4 | Arka sağ motor PWM | PWM, 50Hz |
| 20 | Servo 1 | Aileron kontrol | PWM, 50Hz |
| 21 | Servo 2 | Elevator kontrol | PWM, 50Hz |
| 22 | Servo 3 | Rudder kontrol | PWM, 50Hz |
| 25 | Status LED | Sistem durumu (dahili) | Digital Output |
| 26 | Servo 4 | Throttle kontrol | PWM, 50Hz |
| 28 | External LED | Harici durum LED | Digital Output |

### Pin Atamaları

#### I2C Sensör Veriyolu (I2C0)

- **SDA Pin**: GPIO 4 - I2C veri hattı
- **SCL Pin**: GPIO 5 - I2C saat hattı
- **Hız**: 400 kHz

**Bağlı Sensörler:**

- **BMI270 IMU** (6-axis gyro + accelerometer)
  - I2C Adresi: 0x68
  - Gyro Aralığı: ±2000°/s
  - Accel Aralığı: ±16g
  - Örnekleme Oranı: 1600 Hz
  - Özellikler: Düşük güç, yüksek hassasiyet

- **BMP388 Barometer** (basınç + sıcaklık)
  - I2C Adresi: 0x77
  - Basınç Aralığı: 300-1250 hPa
  - Sıcaklık Aralığı: -40°C ila +85°C
  - Çözünürlük: 0.016 Pa (≈13cm yükseklik)
  - Özellikler: Yüksek hassasiyet, düşük gürültü

- **GPS Modülü** (UART üzerinden)
  - UART1 bağlantısı (GPIO 8/9)
  - NMEA 0183 protokolü
  - Baud Rate: 9600
  - Desteklenen: GPGGA, GPRMC mesajları

#### UART İletişimi

- **UART0 (MAVLink Telemetri)**
  - TX Pin: GPIO 0 - Telemetri verisi çıkışı
  - RX Pin: GPIO 1 - Telemetri verisi girişi
  - Baud Rate: 57600
  - Protokol: MAVLink v2.0
  - Kullanım: Ground station bağlantısı

- **UART1 (GPS Modülü)**
  - TX Pin: GPIO 8 - GPS'e komut gönderimi
  - RX Pin: GPIO 9 - GPS verisi alımı
  - Baud Rate: 9600
  - Protokol: NMEA 0183
  - Desteklenen Mesajlar: $GPGGA, $GPRMC, $GPVTG

#### RC Kumanda Girişi (SBUS)

- **SBUS Pin**: GPIO 13 - RC kumanda verisi girişi (PIO)
- **Protokol**: SBUS (100kbaud, 8E2, inverted)
- **Kanal Sayısı**: 16 kanal (11-bit çözünürlük)
- **Kanal Değer Aralığı**: 0-2047 (tipik 172-1811)
- **Frame Rate**: 14ms (≈72Hz)
- **Özellikler**:
  - Donanım seviyesinde sinyal inversiyonu
  - PIO state machine ile gerçek zamanlı işleme
  - Double buffering ile veri kaybı önleme
  - Failsafe ve frame loss algılama
  - Tek pin üzerinden 16 kanal veri

**Tipik Kanal Atamaları:**

- Kanal 1: Roll (Aileron)
- Kanal 2: Pitch (Elevator)
- Kanal 3: Throttle (Gaz)
- Kanal 4: Yaw (Rudder)
- Kanal 5: Flight Mode
- Kanal 6: Aux 1 (Arm/Disarm)
- Kanal 7-16: Ek fonksiyonlar

#### Motor Çıkışları (PWM)

**Quadcopter X Konfigürasyonu:**

- **Motor 1**: GPIO 16 - Ön sol motor (CCW)
- **Motor 2**: GPIO 17 - Ön sağ motor (CW)
- **Motor 3**: GPIO 18 - Arka sol motor (CW)
- **Motor 4**: GPIO 19 - Arka sağ motor (CCW)

**PWM Parametreleri:**

- **Frekans**: 50 Hz (servo uyumlu)
- **Pulse Width**: 1000-2000 μs
- **Çözünürlük**: 12-bit (4096 seviye)
- **Min Throttle**: 1000 μs (motor durdu)
- **Max Throttle**: 2000 μs (maksimum güç)
- **Güvenlik**: 1500 μs idle değeri

**ESC Uyumluluğu:**

- Standard servo PWM sinyali
- SimonK/BLHeli firmware uyumluluğu
- Oneshot125/Multishot desteklenmiyor (gelecek sürümlerde)

#### LED ve Durum Göstergeleri

- **Status LED**: GPIO 25 (dahili LED) - Sistem durumu
  - Sürekli yanık: Normal çalışma
  - Hızlı yanıp sönme: Arm/disarm durumu
  - Yavaş yanıp sönme: Sensör kalibrasyonu
  - Kapalı: Kritik hata

- **External LED**: GPIO 28 - Harici LED çıkışı
  - Flight mode göstergesi
  - GPS kilidi durumu
  - Fail-safe uyarısı

#### Servo Çıkışları (Sabit Kanat için)

- **Servo 1**: GPIO 20 - Aileron/Roll kontrol
- **Servo 2**: GPIO 21 - Elevator/Pitch kontrol  
- **Servo 3**: GPIO 22 - Rudder/Yaw kontrol
- **Servo 4**: GPIO 26 - Throttle/ESC çıkışı

**Servo Parametreleri:**

- **Frekans**: 50 Hz standart servo PWM
- **Pulse Width**: 1000-2000 μs
- **Orta Nokta**: 1500 μs
- **Travel**: ±500 μs (±60° yaklaşık)

#### Ek GPIO Pinleri

**Gelecek genişlemeler için rezerve:**

- **GPIO 2**: SPI0 SCK - Ek sensörler için
- **GPIO 3**: SPI0 MOSI - Harici flash/SD
- **GPIO 6**: Buzzer çıkışı
- **GPIO 7**: Emergency switch girişi
- **GPIO 10**: CS pin (SPI cihazları için)
- **GPIO 11**: Reserved
- **GPIO 12**: Reserved
- **GPIO 14**: Armed LED çıkışı
- **GPIO 15**: GPS PPS girişi (precision timing)

### Güç Gereksinimleri ve Elektriksel Özellikler

**Güç Girişi:**

- **USB Micro-B**: 5V @ 500mA max
- **VSYS Pin**: 1.8V - 5.5V harici güç
- **3V3 Pin**: 3.3V @ 300mA (regülatör çıkışı)
- **VBUS Pin**: 5V USB güç çıkışı

**Güç Tüketimi:**

- **Çekirdek MCU**: ~30mA @ 125MHz
- **BMI270 IMU**: ~0.7mA (aktif mod)
- **BMP388 Barometer**: ~3.4μA (standby)
- **LED'ler**: ~20mA (tümü aktif)
- **UART İletişimi**: ~5mA
- **Toplam**: ~150mA (tipik çalışma)

**GPIO Elektriksel Sınırlar:**

- **Çıkış Akımı**: 12mA per pin (max)
- **Toplam Çıkış**: 50mA (tüm GPIO)
- **Giriş Voltajı**: 0V - 3.3V
- **Çıkış Voltajı**: 0V - 3.3V (3.3V VCC)

**PWM/Servo Güç:**

- **Servo güç**: Harici 5V beslemesi gerekli
- **Motor ESC**: Harici LiPo/güç kaynağı
- **Maksimum yük**: GPIO başına 12mA

## 📋 Özellikler

### Sensör Desteği

- **IMU**: BMI270 6-axis (gyro + accel)
- **Barometer**: BMP388 (yükseklik + hava basıncı)
- **GPS**: NMEA uyumlu GPS modülleri

### RC Kumanda

- **SBUS Protokolü**: 16 kanala kadar destek
- **Failsafe Algılama**: Sinyal kaybı tespiti
- **Gerçek Zamanlı İşleme**: PIO tabanlı donanım dekodlama

### Kontrol Algoritmaları

- **PID Kontrolörleri**: Roll, pitch, yaw eksenleri için ayrı PID
- **Anti-windup**: Integral sınırlama ile kontrol kararlılığı
- **Motor Karıştırma**: Quadcopter konfigürasyonu için

### Telemetri İletişimi

- **MAVLink v2.0**: Standart drone telemetri protokolü
- **Mesaj Tipleri**:
  - Heartbeat (1 Hz)
  - Attitude (10 Hz)
  - RC Channels Override
  - System Status

### Filtreleme

- **Mahony AHRS**: Sensör füzyonu için
- **Kalman Filtresi**: GPS + IMU entegrasyonu için

## 🔨 Derleme ve Kurulum

### Gereksinimler

- Raspberry Pi Pico SDK
- CMake 3.16+
- MAVLink C Library v2

### Kurulum ve Derleme

1. **MAVLink kütüphanesini klonlayın:**

   ```bash
   git clone https://github.com/mavlink/c_library_v2.git
   ```

1. **Projeyi manuel olarak derleyin:**

   ```bash
   mkdir build
   cd build
   cmake ..
   make -j4
   ```

1. **VS Code ile derleme:**

   - `Ctrl+Shift+P` → "Tasks: Run Task" → "Compile Project"

### Programlama

- **Picotool ile**: `Run Project` task'ını kullanın
- **OpenOCD ile**: `Flash` task'ını kullanın

## 📁 Proje Yapısı

```text
src/
├── main.c              # Ana flight controller kodu (legacy)
├── main.cpp            # BMI270 test kodu (aktif)
├── sbus_reader.*       # RC girişi (PIO tabanlı)
├── pid_controller.*    # PID kontrol algoritmaları
├── mavlink_handler.*   # Telemetri iletişimi
├── sensors/            # Sensör sürücüleri (C++)
│   ├── bmi270.*        # IMU sürücüsü
│   ├── bmp388.*        # Barometer sürücüsü
│   └── gps.*           # GPS sürücüsü
├── control/            # Kontrol algoritmaları
│   ├── mixer.*         # Motor karıştırma
│   └── pid.*           # Gelişmiş PID
├── filter/             # Sinyal işleme
│   └── mahony.*        # AHRS filtresi
└── comms/              # İletişim modülleri
    ├── esp_at.*        # WiFi köprüsü
    └── mavlink_bridge.*# MAVLink yönlendirme
```

## ⚙️ Konfigürasyon

### PID Ayarları

```c
// Örnek PID kazançları (ayarlanması gerekir)
pid_init(&roll_pid, 1.0f, 0.1f, 0.05f);   // Kp, Ki, Kd
pid_init(&pitch_pid, 1.0f, 0.1f, 0.05f);
pid_init(&yaw_pid, 0.8f, 0.05f, 0.02f);
```

### SBUS Konfigürasyonu

```c
#define SBUS_PIN 13
sbus_reader_init(pio0, sm, SBUS_PIN);
```

### I2C Konfigürasyonu

```cpp
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
i2c_init(I2C_PORT, 400 * 1000); // 400 kHz
```

## 🔍 Debug ve Test

### USB Seri İletişimi

- **printf** çıkışı USB üzerinden aktif
- **UART** çıkışı kapalı (MAVLink ile çakışma önleme)

### LED Durum Göstergeleri

- **Sürekli yanık**: Normal çalışma
- **Yanıp sönük**: Sensör hatası
- **Kapalı**: Sistem hatası

## 📡 MAVLink Bağlantısı

### Ground Station Ayarları

- **Baud Rate**: 57600
- **Protokol**: MAVLink v2.0
- **System ID**: 1
- **Component ID**: MAV_COMP_ID_AUTOPILOT1

### Desteklenen Mesajlar

- HEARTBEAT
- ATTITUDE
- RC_CHANNELS_OVERRIDE
- SYS_STATUS

## 🔧 Geliştirme Notları

### PIO Kullanımı

- SBUS okuyucu özel PIO programı kullanır
- 100kbaud 8E2 inverted protokol
- Donanım seviyesinde sinyal inversiyonu

### Timing

- Ana döngü `to_ms_since_boot()` kullanır
- MAVLink heartbeat: 1 Hz
- Attitude telemetri: 10 Hz

### Bellek Yönetimi

- Dinamik bellek tahsisi yok
- Tüm bufferlar statik olarak ayrılmış
- PIO ve ana döngü arasında lock-free iletişim

## 📝 Lisans

Bu proje açık kaynak kodlu olarak geliştirilmektedir.
