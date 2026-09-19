#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <mbedtls/aes.h>
#include <mbedtls/md.h>

// ===== User settings =====
#define NODE_ID 1
#define NETWORK_ID 0x23

// Keep all nodes on the same LoRa settings.
#define LORA_FREQ_MHZ 433.0
#define LORA_BW_KHZ 125.0
#define LORA_SF 9
#define LORA_CR 5
#define LORA_SYNC_WORD 0x12
#define LORA_POWER_DBM 17
#define LORA_PREAMBLE_LEN 8

#define DEFAULT_TTL 4
#define MAX_PAYLOAD_LEN 120
#define MAX_INPUT_LEN 180
#define BROADCAST_ID 255
#define MAX_FRAME_LEN (sizeof(MeshHeader) + MAX_PAYLOAD_LEN)

// Packet types. Private payloads are encrypted end to end.
#define PACKET_TYPE_TEXT 1
#define PACKET_TYPE_PRIVATE 2
#define PACKET_TYPE_ACK 3

// Opportunistic forwarding parameters.
#define SEEN_CACHE_SIZE 48
#define FORWARD_QUEUE_SIZE 6
#define SEEN_TTL_MS 180000UL
#define FORWARD_MIN_DELAY_MS 100
#define FORWARD_MAX_DELAY_MS 720
#define ACK_TIMEOUT_MS 1800UL
#define MAX_TX_ATTEMPTS 3

// Private payload wire format: nonce + tag + ciphertext, all hex encoded.
#define ENC_NONCE_LEN 8
#define ENC_TAG_LEN 8
#define ENC_BINARY_OVERHEAD (ENC_NONCE_LEN + ENC_TAG_LEN)
#define ENC_WIRE_OVERHEAD (ENC_BINARY_OVERHEAD * 2)
#define MAX_PRIVATE_TEXT_LEN ((MAX_PAYLOAD_LEN - ENC_WIRE_OVERHEAD) / 2)

#define WIFI_AP_CHANNEL 6
#define WIFI_MAX_CLIENTS 4

// ===== Heltec Wireless Stick V3 + SX1262 pins =====
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_NSS 8
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_DIO1 14

const uint8_t SHARED_SECRET[] = {
  0x4C, 0x6F, 0x52, 0x61, 0x4D, 0x65, 0x73, 0x68,
  0x2D, 0x48, 0x65, 0x6C, 0x74, 0x65, 0x63, 0x2D,
  0x34, 0x33, 0x33, 0x2D, 0x32, 0x30, 0x32, 0x36
};

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
  uint8_t dst;
  uint8_t type;
  uint16_t seq;
  uint32_t seenAt;
};

struct PendingForward {
  bool used;
  uint8_t src;
  uint8_t dst;
  uint8_t type;
  uint16_t seq;
  uint32_t dueAt;
  size_t frameLen;
  uint8_t frame[MAX_FRAME_LEN];
};

SPIClass loraSpi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
WebServer server(80);
DNSServer dnsServer;
WiFiServer tcpServer(8266);
WiFiClient tcpClient;

SeenPacket seen[SEEN_CACHE_SIZE];
PendingForward forwardQueue[FORWARD_QUEUE_SIZE];
uint16_t nextSeq = 1;
volatile bool rxFlag = false;

String serialLine;
String tcpLine;
String lastTx = "none";
String lastRx = "none";
String lastStatus = "BOOT";
String apSsid;
String logLines[8];
uint8_t logHead = 0;
uint8_t logCount = 0;
float lastRssi = 0.0f;
float lastSnr = 0.0f;
uint32_t tcpLastByteAt = 0;

volatile bool ackReceived = false;
uint8_t ackSource = 0;
uint16_t ackSequence = 0;

void IRAM_ATTR setRxFlag() {
  rxFlag = true;
}

bool timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

String escapeJson(const String &input) {
  String out;
  out.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<uint8_t>(c) < 0x20) {
          out += ' ';
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

// Do not split a UTF-8 codepoint when enforcing the LoRa payload limit.
String clipUtf8Prefix(const String &input, size_t maxBytes) {
  if (input.length() <= maxBytes) {
    return input;
  }

  String out = input.substring(0, maxBytes);
  while (out.length() > 0) {
    uint8_t tail = static_cast<uint8_t>(out[out.length() - 1]);
    if ((tail & 0x80) == 0) {
      break;
    }

    size_t start = out.length() - 1;
    while (start > 0 && ((static_cast<uint8_t>(out[start]) & 0xC0) == 0x80)) {
      start--;
    }

    uint8_t lead = static_cast<uint8_t>(out[start]);
    size_t expected = 1;
    if ((lead & 0xE0) == 0xC0) {
      expected = 2;
    } else if ((lead & 0xF0) == 0xE0) {
      expected = 3;
    } else if ((lead & 0xF8) == 0xF0) {
      expected = 4;
    }

    if (start + expected <= out.length()) {
      break;
    }
    out.remove(start);
  }
  return out;
}

char hexDigit(uint8_t value) {
  value &= 0x0F;
  return value < 10 ? static_cast<char>('0' + value) : static_cast<char>('A' + value - 10);
}

void appendHexByte(String &out, uint8_t value) {
  out += hexDigit(value >> 4);
  out += hexDigit(value);
}

bool decodeHexDigit(char c, uint8_t &value) {
  if (c >= '0' && c <= '9') {
    value = static_cast<uint8_t>(c - '0');
    return true;
  }
  if (c >= 'a' && c <= 'f') {
    value = static_cast<uint8_t>(c - 'a' + 10);
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    value = static_cast<uint8_t>(c - 'A' + 10);
    return true;
  }
  return false;
}

bool hexToBytes(const String &hex, uint8_t *out, size_t outMax, size_t &outLen) {
  if ((hex.length() % 2) != 0 || (hex.length() / 2) > outMax) {
    return false;
  }

  outLen = 0;
  for (size_t i = 0; i < hex.length(); i += 2) {
    uint8_t high = 0;
    uint8_t low = 0;
    if (!decodeHexDigit(hex[i], high) || !decodeHexDigit(hex[i + 1], low)) {
      return false;
    }
    out[outLen++] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

void pushLog(const String &line) {
  String entry = String(millis() / 1000) + "s " + line;
  logLines[logHead] = entry;
  logHead = (logHead + 1) % 8;
  if (logCount < 8) {
    logCount++;
  }
}

String buildLogsJson() {
  String out = "[";
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t index = (logHead + 8 - logCount + i) % 8;
    if (i > 0) {
      out += ",";
    }
    out += "\"";
    out += escapeJson(logLines[index]);
    out += "\"";
  }
  out += "]";
  return out;
}

bool alreadySeen(uint8_t src, uint8_t dst, uint8_t type, uint16_t seq) {
  uint32_t now = millis();
  int8_t replacement = -1;
  uint32_t oldestAge = 0;

  for (uint8_t i = 0; i < SEEN_CACHE_SIZE; i++) {
    if (seen[i].src == src && seen[i].dst == dst &&
        seen[i].type == type && seen[i].seq == seq &&
        now - seen[i].seenAt < SEEN_TTL_MS) {
      return true;
    }

    uint32_t age = now - seen[i].seenAt;
    if (replacement < 0 || age > oldestAge) {
      replacement = static_cast<int8_t>(i);
      oldestAge = age;
    }
  }

  if (replacement < 0) {
    replacement = 0;
  }
  seen[replacement] = {src, dst, type, seq, now};
  return false;
}

void cancelPendingForward(uint8_t src, uint8_t dst, uint8_t type,
                          uint16_t seq) {
  for (uint8_t i = 0; i < FORWARD_QUEUE_SIZE; i++) {
    if (forwardQueue[i].used && forwardQueue[i].src == src &&
        forwardQueue[i].dst == dst && forwardQueue[i].type == type &&
        forwardQueue[i].seq == seq) {
      forwardQueue[i].used = false;
      pushLog(String("SUPPRESS src=") + src + " seq=" + seq);
    }
  }
}

uint16_t forwardingDelayMs(int16_t rssi, float snr) {
  // Weak receivers relay early; strong receivers wait and often hear a
  // duplicate first. This creates a distributed relay priority without a hub.
  int16_t quality = constrain(static_cast<int16_t>(rssi + 120), 0, 70);
  int16_t snrBonus = constrain(static_cast<int16_t>(snr * 8.0f), -80, 80);
  int32_t delayMs = 360 + quality * 4 + snrBonus;
  delayMs += random(-70, 71);
  return static_cast<uint16_t>(constrain(delayMs,
                                         static_cast<int32_t>(FORWARD_MIN_DELAY_MS),
                                         static_cast<int32_t>(FORWARD_MAX_DELAY_MS)));
}

bool queueForward(const uint8_t *frame, size_t frameLen, int16_t rssi, float snr) {
  if (frameLen < sizeof(MeshHeader) ||
      frameLen > MAX_FRAME_LEN) {
    return false;
  }

  MeshHeader header;
  memcpy(&header, frame, sizeof(header));
  if (header.ttl <= 1) {
    return false;
  }

  int8_t freeSlot = -1;
  for (uint8_t i = 0; i < FORWARD_QUEUE_SIZE; i++) {
    if (!forwardQueue[i].used) {
      freeSlot = static_cast<int8_t>(i);
      break;
    }
  }
  if (freeSlot < 0) {
    pushLog("FWD queue full");
    return false;
  }

  header.ttl--;
  memcpy(forwardQueue[freeSlot].frame, frame, frameLen);
  memcpy(forwardQueue[freeSlot].frame, &header, sizeof(header));
  forwardQueue[freeSlot].used = true;
  forwardQueue[freeSlot].src = header.src;
  forwardQueue[freeSlot].dst = header.dst;
  forwardQueue[freeSlot].type = header.type;
  forwardQueue[freeSlot].seq = header.seq;
  forwardQueue[freeSlot].frameLen = frameLen;
  forwardQueue[freeSlot].dueAt = millis() + forwardingDelayMs(rssi, snr);
  return true;
}

void startReceive() {
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    lastStatus = String("RX start failed: ") + state;
    pushLog(lastStatus);
  }
}

bool transmitFrame(const uint8_t *frame, size_t frameLen, uint16_t randomMin,
                   uint16_t randomMax, const char *logPrefix) {
  radio.standby();
  if (randomMax > randomMin) {
    delay(random(randomMin, randomMax));
  }

  int state = radio.transmit(frame, frameLen);
  startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    lastStatus = String(logPrefix) + " failed: " + state;
    pushLog(lastStatus);
    return false;
  }

  pushLog(String(logPrefix) + " OK");
  return true;
}

void processForwardQueue() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < FORWARD_QUEUE_SIZE; i++) {
    if (!forwardQueue[i].used || !timeReached(now, forwardQueue[i].dueAt)) {
      continue;
    }

    PendingForward &pending = forwardQueue[i];
    pending.used = false;
    if (transmitFrame(pending.frame, pending.frameLen, 0, 0, "FWD")) {
      MeshHeader header;
      memcpy(&header, pending.frame, sizeof(header));
      pushLog(String("FWD src=") + header.src + " seq=" + header.seq +
              " ttl=" + header.ttl);
    }
    return;
  }
}

void derivePrivateKey(uint8_t src, uint8_t dst, uint16_t seq,
                      uint8_t *keyOut, size_t keyLen) {
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_setup(&ctx, info, 1);
  mbedtls_md_hmac_starts(&ctx, SHARED_SECRET, sizeof(SHARED_SECRET));
  mbedtls_md_hmac_update(&ctx,
                         reinterpret_cast<const uint8_t *>("LoRaMeshPrivateV1"), 17);
  mbedtls_md_hmac_update(&ctx, &src, 1);
  mbedtls_md_hmac_update(&ctx, &dst, 1);
  uint8_t seqBytes[2] = {
    static_cast<uint8_t>(seq & 0xFF),
    static_cast<uint8_t>((seq >> 8) & 0xFF)
  };
  mbedtls_md_hmac_update(&ctx, seqBytes, sizeof(seqBytes));

  uint8_t digest[32];
  mbedtls_md_hmac_finish(&ctx, digest);
  mbedtls_md_free(&ctx);
  memcpy(keyOut, digest, keyLen);
}

void makePrivateTag(uint8_t src, uint8_t dst, uint16_t seq,
                    const uint8_t *nonce, const uint8_t *cipher,
                    size_t cipherLen, uint8_t *tagOut) {
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_setup(&ctx, info, 1);
  mbedtls_md_hmac_starts(&ctx, SHARED_SECRET, sizeof(SHARED_SECRET));
  mbedtls_md_hmac_update(&ctx,
                         reinterpret_cast<const uint8_t *>("LoRaMeshTagV1"), 13);
  mbedtls_md_hmac_update(&ctx, &src, 1);
  mbedtls_md_hmac_update(&ctx, &dst, 1);
  uint8_t seqBytes[2] = {
    static_cast<uint8_t>(seq & 0xFF),
    static_cast<uint8_t>((seq >> 8) & 0xFF)
  };
  uint8_t lenByte = static_cast<uint8_t>(cipherLen);
  mbedtls_md_hmac_update(&ctx, seqBytes, sizeof(seqBytes));
  mbedtls_md_hmac_update(&ctx, &lenByte, 1);
  mbedtls_md_hmac_update(&ctx, nonce, ENC_NONCE_LEN);
  mbedtls_md_hmac_update(&ctx, cipher, cipherLen);

  uint8_t digest[32];
  mbedtls_md_hmac_finish(&ctx, digest);
  mbedtls_md_free(&ctx);
  memcpy(tagOut, digest, ENC_TAG_LEN);
}

void cryptPrivatePayload(uint8_t src, uint8_t dst, uint16_t seq,
                         const uint8_t *nonce, const uint8_t *input,
                         uint8_t *output, size_t len) {
  uint8_t key[16];
  derivePrivateKey(src, dst, seq, key, sizeof(key));

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, key, 128);

  uint8_t streamBlock[16];
  uint8_t counter[16];
  for (size_t offset = 0; offset < len; offset += sizeof(streamBlock)) {
    memset(counter, 0, sizeof(counter));
    counter[0] = src;
    counter[1] = dst;
    counter[2] = static_cast<uint8_t>(seq & 0xFF);
    counter[3] = static_cast<uint8_t>((seq >> 8) & 0xFF);
    memcpy(counter + 4, nonce, ENC_NONCE_LEN);
    uint32_t blockIndex = offset / sizeof(streamBlock);
    counter[12] = static_cast<uint8_t>(blockIndex & 0xFF);
    counter[13] = static_cast<uint8_t>((blockIndex >> 8) & 0xFF);
    counter[14] = static_cast<uint8_t>((blockIndex >> 16) & 0xFF);
    counter[15] = static_cast<uint8_t>((blockIndex >> 24) & 0xFF);
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, counter, streamBlock);

    size_t blockLen = min(sizeof(streamBlock), len - offset);
    for (size_t i = 0; i < blockLen; i++) {
      output[offset + i] = input[offset + i] ^ streamBlock[i];
    }
  }
  mbedtls_aes_free(&aes);
}

bool encryptPrivateText(uint8_t dst, uint16_t seq, const String &plain,
                        String &encryptedOut) {
  String clipped = clipUtf8Prefix(plain, MAX_PRIVATE_TEXT_LEN);
  size_t plainLen = clipped.length();
  if (dst == BROADCAST_ID || plainLen == 0 || plainLen > MAX_PRIVATE_TEXT_LEN) {
    return false;
  }

  uint8_t nonce[ENC_NONCE_LEN];
  for (uint8_t i = 0; i < ENC_NONCE_LEN; i += 4) {
    uint32_t value = esp_random();
    nonce[i] = static_cast<uint8_t>(value & 0xFF);
    nonce[i + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    nonce[i + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    nonce[i + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
  }

  uint8_t cipher[MAX_PRIVATE_TEXT_LEN];
  cryptPrivatePayload(NODE_ID, dst, seq, nonce,
                      reinterpret_cast<const uint8_t *>(clipped.c_str()),
                      cipher, plainLen);

  uint8_t tag[ENC_TAG_LEN];
  makePrivateTag(NODE_ID, dst, seq, nonce, cipher, plainLen, tag);

  encryptedOut = "";
  encryptedOut.reserve(ENC_WIRE_OVERHEAD + plainLen * 2);
  for (uint8_t i = 0; i < ENC_NONCE_LEN; i++) {
    appendHexByte(encryptedOut, nonce[i]);
  }
  for (uint8_t i = 0; i < ENC_TAG_LEN; i++) {
    appendHexByte(encryptedOut, tag[i]);
  }
  for (size_t i = 0; i < plainLen; i++) {
    appendHexByte(encryptedOut, cipher[i]);
  }
  return true;
}

bool decryptPrivateText(uint8_t src, uint8_t dst, uint16_t seq,
                        const String &encrypted, String &plainOut) {
  if (encrypted.length() <= ENC_WIRE_OVERHEAD || dst != NODE_ID) {
    return false;
  }

  uint8_t raw[ENC_BINARY_OVERHEAD + MAX_PRIVATE_TEXT_LEN];
  size_t rawLen = 0;
  if (!hexToBytes(encrypted, raw, sizeof(raw), rawLen) ||
      rawLen <= ENC_BINARY_OVERHEAD) {
    return false;
  }

  const uint8_t *nonce = raw;
  const uint8_t *tag = raw + ENC_NONCE_LEN;
  const uint8_t *cipher = raw + ENC_BINARY_OVERHEAD;
  size_t cipherLen = rawLen - ENC_BINARY_OVERHEAD;

  uint8_t expectedTag[ENC_TAG_LEN];
  makePrivateTag(src, dst, seq, nonce, cipher, cipherLen, expectedTag);
  if (memcmp(tag, expectedTag, ENC_TAG_LEN) != 0) {
    return false;
  }

  uint8_t plain[MAX_PRIVATE_TEXT_LEN + 1];
  cryptPrivatePayload(src, dst, seq, nonce, cipher, plain, cipherLen);
  plain[cipherLen] = 0;
  plainOut = reinterpret_cast<const char *>(plain);
  return true;
}

void sendAck(uint8_t dst, uint16_t seq) {
  MeshHeader header;
  header.networkId = NETWORK_ID;
  header.src = NODE_ID;
  header.dst = dst;
  header.seq = seq;
  header.ttl = DEFAULT_TTL;
  header.type = PACKET_TYPE_ACK;
  header.len = 0;

  uint8_t frame[sizeof(MeshHeader)];
  memcpy(frame, &header, sizeof(header));
  transmitFrame(frame, sizeof(frame), 40, 180, "ACK");
}

void handleReceivedPacket() {
  uint8_t frame[sizeof(MeshHeader) + MAX_PAYLOAD_LEN];
  int state = radio.readData(frame, sizeof(frame));
  startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    lastStatus = String("RX failed: ") + state;
    pushLog(lastStatus);
    return;
  }

  size_t frameLen = radio.getPacketLength();
  if (frameLen < sizeof(MeshHeader) ||
      frameLen > sizeof(MeshHeader) + MAX_PAYLOAD_LEN) {
    pushLog("RX bad frame size");
    return;
  }

  MeshHeader header;
  memcpy(&header, frame, sizeof(header));
  if (header.networkId != NETWORK_ID || header.src == NODE_ID) {
    return;
  }
  if (header.len > MAX_PAYLOAD_LEN ||
      frameLen != sizeof(MeshHeader) + header.len) {
    pushLog("RX bad length");
    return;
  }

  if (alreadySeen(header.src, header.dst, header.type, header.seq)) {
    // A lost ACK can cause the sender to retry a valid private packet.
    // Acknowledge it again, but never display or relay the duplicate.
    if (header.type == PACKET_TYPE_PRIVATE && header.dst == NODE_ID) {
      String duplicatePayload;
      duplicatePayload.reserve(header.len);
      for (uint8_t i = 0; i < header.len; i++) {
        duplicatePayload += static_cast<char>(frame[sizeof(MeshHeader) + i]);
      }

      String duplicatePlain;
      if (decryptPrivateText(header.src, header.dst, header.seq,
                             duplicatePayload, duplicatePlain)) {
        sendAck(header.src, header.seq);
      }
    }
    cancelPendingForward(header.src, header.dst, header.type, header.seq);
    return;
  }

  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();

  if (header.type == PACKET_TYPE_ACK) {
    if (header.dst == NODE_ID) {
      ackReceived = true;
      ackSource = header.src;
      ackSequence = header.seq;
      lastStatus = "ACK RX";
      pushLog(String("ACK src=") + header.src + " seq=" + header.seq);
    }
  } else {
    String payload;
    payload.reserve(header.len);
    for (uint8_t i = 0; i < header.len; i++) {
      payload += static_cast<char>(frame[sizeof(MeshHeader) + i]);
    }

    bool accepted = false;
    if (header.type == PACKET_TYPE_PRIVATE) {
      if (header.dst == NODE_ID) {
        String plain;
        accepted = decryptPrivateText(header.src, header.dst, header.seq,
                                       payload, plain);
        if (accepted) {
          lastRx = String("PRV src ") + header.src + ": " + plain;
          lastStatus = "PRIVATE RX";
          sendAck(header.src, header.seq);
        } else {
          lastRx = String("PRV src ") + header.src + ": decrypt fail";
          lastStatus = "DECRYPT FAIL";
        }
      } else {
        lastRx = String("ENC src ") + header.src + " dst " + header.dst;
        lastStatus = "ENC RELAY";
        accepted = true;
      }
    } else if (header.type == PACKET_TYPE_TEXT) {
      lastRx = String("src ") + header.src + ": " + payload;
      lastStatus = "RX OK";
      accepted = true;
    }

    if (accepted) {
      pushLog(String("RX ") + lastRx + " RSSI=" +
              String(lastRssi, 1) + " SNR=" + String(lastSnr, 1));
    }
  }

  // The destination consumes the packet. Every other node may relay it.
  if (header.dst != NODE_ID && header.ttl > 1) {
    queueForward(frame, frameLen, static_cast<int16_t>(lastRssi), lastSnr);
  }
}

String parseUserMessage(const String &input, uint8_t &dst) {
  dst = BROADCAST_ID;
  String text = input;
  text.trim();
  if (text.length() == 0) {
    return "";
  }

  if (text.startsWith("@")) {
    int spaceIndex = text.indexOf(' ');
    if (spaceIndex > 1) {
      String idText = text.substring(1, spaceIndex);
      int maybeDst = idText.toInt();
      if (maybeDst >= 0 && maybeDst <= 254) {
        dst = static_cast<uint8_t>(maybeDst);
        text = text.substring(spaceIndex + 1);
        text.trim();
      }
    }
  }
  return clipUtf8Prefix(text, MAX_PAYLOAD_LEN);
}

bool sendUserPacket(uint8_t dst, uint8_t type, const String &wirePayload,
                    uint16_t seq, const String &displayLabel) {
  String clipped = clipUtf8Prefix(wirePayload, MAX_PAYLOAD_LEN);
  uint8_t len = static_cast<uint8_t>(clipped.length());
  if (len == 0) {
    return false;
  }

  uint8_t frame[sizeof(MeshHeader) + MAX_PAYLOAD_LEN];
  MeshHeader header;
  header.networkId = NETWORK_ID;
  header.src = NODE_ID;
  header.dst = dst;
  header.seq = seq;
  header.ttl = DEFAULT_TTL;
  header.type = type;
  header.len = len;
  memcpy(frame, &header, sizeof(header));
  memcpy(frame + sizeof(header), clipped.c_str(), len);

  bool sent = transmitFrame(frame, sizeof(header) + len, 20, 160, "TX");
  if (sent) {
    lastTx = displayLabel;
    lastStatus = "TX OK";
    pushLog(String("TX seq=") + seq + " " + displayLabel);
  }
  return sent;
}

bool waitForAck(uint8_t expectedSource, uint16_t seq) {
  uint32_t deadline = millis() + ACK_TIMEOUT_MS;
  while (!timeReached(millis(), deadline)) {
    if (rxFlag) {
      rxFlag = false;
      handleReceivedPacket();
      if (ackReceived && ackSource == expectedSource && ackSequence == seq) {
        ackReceived = false;
        return true;
      }
    }
    processForwardQueue();
    delay(2);
  }
  return false;
}

void transmitUserText(const String &input, const String &sourceTag) {
  uint8_t dst = BROADCAST_ID;
  String payload = parseUserMessage(input, dst);
  if (payload.length() == 0) {
    return;
  }

  uint16_t seq = nextSeq++;
  uint8_t packetType = PACKET_TYPE_TEXT;
  String wirePayload = payload;
  String displayLabel = dst == BROADCAST_ID
                          ? String("BCAST ") + payload
                          : String("DST ") + dst + " " + payload;

  if (dst != BROADCAST_ID) {
    payload = clipUtf8Prefix(payload, MAX_PRIVATE_TEXT_LEN);
    if (!encryptPrivateText(dst, seq, payload, wirePayload)) {
      lastStatus = "ENC FAIL";
      pushLog("Encrypt failed");
      return;
    }
    packetType = PACKET_TYPE_PRIVATE;
    displayLabel = String("ENC->") + dst + " " + payload;
  }

  lastStatus = sourceTag + " TX";
  bool delivered = false;
  for (uint8_t attempt = 1; attempt <= MAX_TX_ATTEMPTS; attempt++) {
    ackReceived = false;
    if (!sendUserPacket(dst, packetType, wirePayload, seq, displayLabel)) {
      break;
    }

    if (dst == BROADCAST_ID) {
      delivered = true;
      break;
    }

    if (waitForAck(dst, seq)) {
      delivered = true;
      lastStatus = "DELIVERED";
      pushLog(String("DELIVERED attempt=") + attempt);
      break;
    }

    if (attempt < MAX_TX_ATTEMPTS) {
      lastStatus = String("RETRY ") + (attempt + 1);
      pushLog(lastStatus);
      delay(80 + random(0, 180));
    }
  }

  if (!delivered && dst != BROADCAST_ID) {
    lastStatus = "NO ACK";
    pushLog(String("NO ACK seq=") + seq);
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
        transmitUserText(serialLine, "SER");
      }
      serialLine = "";
    } else if (serialLine.length() < MAX_INPUT_LEN) {
      serialLine += c;
    }
  }
}

void handleTcpInput() {
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      tcpClient = newClient;
      tcpLine = "";
      tcpLastByteAt = millis();
      pushLog("TCP connected");
    }
  }

  while (tcpClient && tcpClient.connected() && tcpClient.available()) {
    char c = static_cast<char>(tcpClient.read());
    tcpLastByteAt = millis();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      tcpLine.trim();
      if (tcpLine.length() > 0) {
        transmitUserText(tcpLine, "TCP");
      }
      tcpLine = "";
    } else if (tcpLine.length() < MAX_INPUT_LEN) {
      tcpLine += c;
    }
  }

  if (tcpClient && tcpClient.connected() && tcpLine.length() > 0 &&
      millis() - tcpLastByteAt > 350) {
    tcpLine.trim();
    if (tcpLine.length() > 0) {
      transmitUserText(tcpLine, "TCP");
      tcpClient.println("OK");
    }
    tcpLine = "";
  }
}

String buildStatusJson() {
  String json;
  json.reserve(620);
  json += "{";
  json += "\"status\":\"";
  json += escapeJson(lastStatus);
  json += "\",\"ssid\":\"";
  json += escapeJson(apSsid);
  json += "\",\"ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\",\"tx\":\"";
  json += escapeJson(lastTx);
  json += "\",\"rx\":\"";
  json += escapeJson(lastRx);
  json += "\",\"rssi\":";
  json += String(lastRssi, 1);
  json += ",\"snr\":";
  json += String(lastSnr, 1);
  json += ",\"logs\":";
  json += buildLogsJson();
  json += "}";
  return json;
}

String buildIndexPage() {
  String page;
  page.reserve(2400);
  page += F("<!doctype html><html><head><meta charset='utf-8'>");
  page += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>LoRa Mesh Terminal</title><style>");
  page += F("body{font-family:system-ui,Arial,sans-serif;margin:16px;max-width:760px;}");
  page += F("h1{font-size:24px;margin:0 0 12px 0;}");
  page += F(".line{margin:6px 0;color:#333;font-size:14px;}");
  page += F(".row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;}");
  page += F("input{flex:1 1 260px;font-size:18px;padding:10px;box-sizing:border-box;}");
  page += F("button{font-size:18px;padding:10px 16px;}");
  page += F("pre{white-space:pre-wrap;word-break:break-word;border:1px solid #ccc;");
  page += F("padding:12px;min-height:180px;background:#f8f8f8;}");
  page += F("</style></head><body><h1>LoRa Mesh Terminal</h1>");
  page += F("<div class='line'>SSID: <span id='ssid'>-</span></div>");
  page += F("<div class='line'>IP: <span id='ip'>-</span></div>");
  page += F("<div class='row'><input id='msg' type='text' maxlength='180' ");
  page += F("placeholder='Message or @nodeId message'></div>");
  page += F("<div class='row' style='margin-top:8px;'><button onclick='sendMsg()'>Send</button>");
  page += F("<span id='state'></span></div><pre id='log'></pre><script>");
  page += F("async function refresh(){const r=await fetch('/status',{cache:'no-store'});");
  page += F("const j=await r.json();document.getElementById('ssid').textContent=j.ssid;");
  page += F("document.getElementById('ip').textContent=j.ip;");
  page += F("document.getElementById('state').textContent=j.status+' | TX '+j.tx+' | RX '+j.rx;");
  page += F("document.getElementById('log').textContent=(j.logs||[]).join('\\n');}");
  page += F("async function sendMsg(){const el=document.getElementById('msg');");
  page += F("const msg=el.value.trim();if(!msg)return;const form=new URLSearchParams();");
  page += F("form.set('msg',msg);document.getElementById('state').textContent='sending...';");
  page += F("await fetch('/send',{method:'POST',headers:{'Content-Type':");
  page += F("'application/x-www-form-urlencoded; charset=UTF-8'},body:form.toString()});");
  page += F("el.value='';await refresh();el.focus();}");
  page += F("document.getElementById('msg').addEventListener('keydown',e=>{");
  page += F("if(e.key==='Enter'){e.preventDefault();sendMsg();}});");
  page += F("refresh();setInterval(refresh,2000);</script></body></html>");
  return page;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", buildIndexPage());
}

void handleSend() {
  String msg = server.hasArg("msg") ? server.arg("msg") : "";
  msg.trim();
  if (msg.length() == 0) {
    server.send(400, "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"empty\"}");
    return;
  }

  transmitUserText(msg, "WEB");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", buildStatusJson());
}

void handleNotFound() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void startWiFiPortal() {
  char ssidBuf[32];
  snprintf(ssidBuf, sizeof(ssidBuf), "LoRaMesh-%02d", NODE_ID);
  apSsid = ssidBuf;

  IPAddress apIp(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIp, gateway, subnet);
  bool apOk = WiFi.softAP(apSsid.c_str(), "00000000",
                          WIFI_AP_CHANNEL, false, WIFI_MAX_CLIENTS);
  if (apOk) {
    dnsServer.start(53, "*", WiFi.softAPIP());
    lastStatus = "AP READY";
    pushLog(String("AP ") + apSsid + " " + WiFi.softAPIP().toString());
  } else {
    lastStatus = "AP FAIL";
    pushLog("AP start failed");
  }
}

void startRadio() {
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                          LORA_SYNC_WORD, LORA_POWER_DBM, LORA_PREAMBLE_LEN);
  if (state != RADIOLIB_ERR_NONE) {
    lastStatus = String("RADIO FAIL ") + state;
    pushLog(lastStatus);
    while (true) {
      delay(1000);
    }
  }

  radio.setDio1Action(setRxFlag);
  startReceive();
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  randomSeed(esp_random());
  memset(seen, 0, sizeof(seen));
  memset(forwardQueue, 0, sizeof(forwardQueue));

  pushLog("BOOT");
  startRadio();
  startWiFiPortal();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/send", HTTP_GET, handleSend);
  server.on("/send", HTTP_POST, handleSend);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  tcpServer.begin();

  lastStatus = "READY";
  pushLog("READY");
  Serial.println();
  Serial.println("LoRaMesh_Terminal_AP ready");
  Serial.print("AP SSID: ");
  Serial.println(apSsid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  handleSerialInput();
  handleTcpInput();

  if (rxFlag) {
    rxFlag = false;
    handleReceivedPacket();
  }
  processForwardQueue();
}
