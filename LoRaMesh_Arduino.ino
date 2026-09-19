/*
  Simple LoRa flood mesh for Heltec Wireless Stick V3 / similar ESP32-S3 + SX1262 boards.

  Arduino IDE dependencies:
  - Board package: esp32 by Espressif Systems
  - Library: RadioLib by Jan Gromes

  For each board, change NODE_ID to a different value before uploading.
  Open Serial Monitor at 115200 baud. Type text and press Enter to send it.
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// ===== User settings =====
#define NODE_ID 1
#define NETWORK_ID 0x23

// Mainland China 470 MHz test frequency. Keep all nodes on the same value.
// Change this only if your hardware and local rules require another band.
#define LORA_FREQ_MHZ 470.125

// More range: increase SF, less airtime: decrease SF.
#define LORA_BW_KHZ 125.0
#define LORA_SF 9
#define LORA_CR 5
#define LORA_SYNC_WORD 0x12
#define LORA_POWER_DBM 17
#define LORA_PREAMBLE_LEN 8

#define DEFAULT_TTL 4
#define MAX_PAYLOAD_LEN 120
#define SEEN_CACHE_SIZE 32

// ===== Heltec ESP32-S3 + SX1262 pin map =====
// This pin map matches common Heltec V3 SX1262 boards. If your Wireless Stick V3
// schematic differs, edit only this block.
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_NSS 8
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_DIO1 14

SPIClass loraSpi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);

struct __attribute__((packed)) MeshHeader {
  uint8_t networkId;
  uint8_t src;
  uint8_t dst;
  uint16_t seq;
  uint8_t ttl;
  uint8_t type;
  uint8_t len;
};

struct SeenPacket {
  uint8_t src;
  uint16_t seq;
  uint32_t seenAt;
};

SeenPacket seen[SEEN_CACHE_SIZE];
uint8_t seenWrite = 0;
uint16_t nextSeq = 1;
volatile bool rxFlag = false;
String serialLine;

void setRxFlag() {
  rxFlag = true;
}

bool alreadySeen(uint8_t src, uint16_t seq) {
  for (uint8_t i = 0; i < SEEN_CACHE_SIZE; i++) {
    if (seen[i].src == src && seen[i].seq == seq) {
      return true;
    }
  }

  seen[seenWrite] = {src, seq, millis()};
  seenWrite = (seenWrite + 1) % SEEN_CACHE_SIZE;
  return false;
}

void printHexByte(uint8_t v) {
  if (v < 16) {
    Serial.print('0');
  }
  Serial.print(v, HEX);
}

void startReceive() {
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("startReceive failed: "));
    Serial.println(state);
  }
}

void sendPacket(uint8_t dst, const uint8_t *payload, uint8_t len, uint8_t ttl, uint16_t seq) {
  uint8_t frame[sizeof(MeshHeader) + MAX_PAYLOAD_LEN];

  MeshHeader header;
  header.networkId = NETWORK_ID;
  header.src = NODE_ID;
  header.dst = dst;
  header.seq = seq;
  header.ttl = ttl;
  header.type = 1;
  header.len = len;

  memcpy(frame, &header, sizeof(header));
  memcpy(frame + sizeof(header), payload, len);

  radio.standby();
  delay(random(20, 160));

  int state = radio.transmit(frame, sizeof(header) + len);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("TX seq="));
    Serial.print(seq);
    Serial.print(F(" dst="));
    Serial.print(dst == 255 ? F("broadcast") : String(dst));
    Serial.print(F(" len="));
    Serial.println(len);
  } else {
    Serial.print(F("TX failed: "));
    Serial.println(state);
  }

  startReceive();
}

void forwardPacket(uint8_t *frame, size_t frameLen) {
  MeshHeader *header = reinterpret_cast<MeshHeader *>(frame);
  if (header->ttl <= 1) {
    return;
  }

  header->ttl--;
  delay(random(80, 420));

  radio.standby();
  int state = radio.transmit(frame, frameLen);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("Forwarded src="));
    Serial.print(header->src);
    Serial.print(F(" seq="));
    Serial.print(header->seq);
    Serial.print(F(" ttl="));
    Serial.println(header->ttl);
  } else {
    Serial.print(F("Forward failed: "));
    Serial.println(state);
  }

  startReceive();
}

void handleReceivedPacket() {
  uint8_t frame[sizeof(MeshHeader) + MAX_PAYLOAD_LEN];
  int state = radio.readData(frame, sizeof(frame));
  startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("RX failed: "));
    Serial.println(state);
    return;
  }

  size_t frameLen = radio.getPacketLength();
  if (frameLen < sizeof(MeshHeader)) {
    Serial.println(F("RX too short"));
    return;
  }

  MeshHeader header;
  memcpy(&header, frame, sizeof(header));

  if (header.networkId != NETWORK_ID) {
    return;
  }
  if (header.src == NODE_ID) {
    return;
  }
  if (header.len > MAX_PAYLOAD_LEN || frameLen != sizeof(MeshHeader) + header.len) {
    Serial.println(F("RX bad length"));
    return;
  }
  if (alreadySeen(header.src, header.seq)) {
    return;
  }

  Serial.print(F("RX from="));
  Serial.print(header.src);
  Serial.print(F(" seq="));
  Serial.print(header.seq);
  Serial.print(F(" ttl="));
  Serial.print(header.ttl);
  Serial.print(F(" rssi="));
  Serial.print(radio.getRSSI());
  Serial.print(F(" snr="));
  Serial.print(radio.getSNR());
  Serial.print(F(" data=\""));
  for (uint8_t i = 0; i < header.len; i++) {
    char c = static_cast<char>(frame[sizeof(MeshHeader) + i]);
    Serial.print(isPrintable(c) ? c : '.');
  }
  Serial.println('"');

  if (header.dst == 255 || header.dst == NODE_ID) {
    Serial.print(F("Payload hex: "));
    for (uint8_t i = 0; i < header.len; i++) {
      printHexByte(frame[sizeof(MeshHeader) + i]);
      Serial.print(' ');
    }
    Serial.println();
  }

  if (header.dst != NODE_ID) {
    forwardPacket(frame, frameLen);
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      serialLine.trim();
      if (serialLine.length() > 0) {
        uint8_t payload[MAX_PAYLOAD_LEN];
        uint8_t len = min(static_cast<int>(serialLine.length()), MAX_PAYLOAD_LEN);
        memcpy(payload, serialLine.c_str(), len);
        sendPacket(255, payload, len, DEFAULT_TTL, nextSeq++);
      }
      serialLine = "";
    } else if (serialLine.length() < MAX_PAYLOAD_LEN) {
      serialLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println(F("LoRaMesh_Arduino starting"));
  Serial.print(F("NODE_ID="));
  Serial.println(NODE_ID);

  randomSeed(esp_random());
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD,
                          LORA_POWER_DBM, LORA_PREAMBLE_LEN);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Radio begin failed: "));
    Serial.println(state);
    Serial.println(F("Check board target, LoRa pins, antenna, and RadioLib install."));
    while (true) {
      delay(1000);
    }
  }

  radio.setDio1Action(setRxFlag);
  startReceive();

  Serial.println(F("Ready. Type a message and press Enter."));
}

void loop() {
  handleSerialInput();

  if (rxFlag) {
    rxFlag = false;
    handleReceivedPacket();
  }
}
