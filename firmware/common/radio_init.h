#pragma once

// Shared nRF24 bring-up helpers.
//
// The STM32F401/F411 boards used by NanoLink can leave SPI1 disabled after
// SPI.begin()/SPI.beginTransaction(). firmware/common/spi_fix.py patches the
// driver so every transfer repairs the peripheral; the functions below add a
// second layer of protection and make radio start-up retry instead of failing
// permanently on a single bad attempt.

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

namespace nl_radio {

inline void repairSpi1() {
#if defined(SPI1)
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // PA5 = SCK, PA6 = MISO, PA7 = MOSI, all alternate function 5.
  if (((GPIOA->MODER >> 10) & 3u) != 2u) {
    GPIOA->MODER = (GPIOA->MODER & ~((3u << 10) | (3u << 12) | (3u << 14)))
                 | ((2u << 10) | (2u << 12) | (2u << 14));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 20) | (0xFu << 24) | (0xFu << 28)))
                  | ((5u << 20) | (5u << 24) | (5u << 28));
  }

  if (!(SPI1->CR1 & SPI_CR1_SPE) || !(SPI1->CR1 & SPI_CR1_MSTR)) {
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (5u << 3) | SPI_CR1_SPE;
  }
#endif
}

// Initialize the radio, retrying a few times, then prove that the SPI link is
// bidirectional by writing the channel register and reading it back.
inline bool begin(RF24 &radio, uint8_t channel, rf24_pa_dbm_e power,
                  uint8_t attempts = 4) {
  for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
    repairSpi1();
    if (radio.begin()) {
      repairSpi1();
      radio.setChannel(channel);
      if (radio.getChannel() == channel) {
        radio.setDataRate(RF24_250KBPS);
        radio.setPALevel(power);
        radio.setAutoAck(false);  // no auto-ACK: every link is fire-and-forget
        radio.setRetries(0, 0);
        return true;
      }
    }
    delay(40);
  }
  return false;
}

// Cheap health check used by the main loop to detect a wedged radio.
inline bool responds(RF24 &radio, uint8_t channel) {
  return radio.getChannel() == channel;
}

}  // namespace nl_radio
