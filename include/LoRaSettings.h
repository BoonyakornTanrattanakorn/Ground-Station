#ifndef LORA_H
#define LORA_H
#include <SX126x.h>
#endif

#ifndef LORA_FREQUENCY
// #define LORA_FREQUENCY 904300000
#define LORA_FREQUENCY 915000000
#endif

#ifndef LORA_POWER
#define LORA_POWER 17
#endif

#ifndef LORA_RX_POWER
#define LORA_RX_POWER SX126X_RX_GAIN_BOOSTED
#endif

#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH SX126X_BW_125000
#endif

#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR 7
#endif

#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE 5
#endif

#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 4
#endif

#ifndef LORA_SYNC_WORD
#define LORA_SYNC_WORD 0x1424
#endif

#ifndef LORA_IQ_INVERTED
#define LORA_IQ_INVERTED true
#endif

#ifndef LORA_NSS_PIN
#define LORA_NSS_PIN 5
#endif

#ifndef LORA_RESET_PIN
#define LORA_RESET_PIN 16
#endif

#ifndef LORA_BUSY_PIN
#define LORA_BUSY_PIN 17
#endif

#ifndef LORA_IRQ_PIN
#define LORA_IRQ_PIN -1
#endif

#ifndef LORA_SCK
#define LORA_SCK_PIN 19
#endif

#ifndef LORA_MISO_PIN
#define LORA_MISO_PIN 18
#endif

#ifndef LORA_MOSI_PIN
#define LORA_MOSI_PIN 23
#endif

#ifndef LORA_DIO3_VOLTAGE
#define LORA_DIO3_VOLTAGE SX126X_DIO3_OUTPUT_1_8
#endif

#ifndef LORA_DIO3_TCXO
#define LORA_DIO3_TCXO SX126X_TCXO_DELAY_1
#endif

#ifndef LORA_LISTEN_MODE
#define LORA_LISTEN_MODE SX126X_RX_SINGLE
#endif

#ifndef LORA_RX_GAIN
#define LORA_RX_GAIN SX126X_RX_GAIN_BOOSTED
#endif

#ifndef LORA_PAYLOAD_LENGTH
#define LORA_PAYLOAD_LENGTH 24
#endif

#ifndef LORA_CRC
#define LORA_CRC true
#endif