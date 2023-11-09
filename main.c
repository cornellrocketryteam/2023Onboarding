#include "f_util.h"
#include "ff.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hw_config.h"
#include "pico/stdlib.h"
#include "rtc.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define FILENAME "log.txt"

#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3

#define NFC_ADDR 0x24
#define SERVO_PIN 15
#define PN532_MIFARE_ISO14443A (0x00)
#define PN532_PREAMBLE (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)
#define PN532_HOSTTOPN532 (0xD4)
#define PN532_I2C_READY (0x01)
#define PN532_COMMAND_INLISTPASSIVETARGET (0x4A)
#define PN532_COMMAND_SAMCONFIGURATION (0x14)

#define PN532_PACKBUFFSIZ 64
uint8_t pn532_packetbuffer[PN532_PACKBUFFSIZ];
uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

/**************************************************************************/
/*!
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes
*/
/**************************************************************************/
void writecommand(uint8_t *cmd, uint8_t cmdlen) {
  // I2C or Serial command write.
  uint8_t packet[8 + cmdlen];
  uint8_t LEN = cmdlen + 1;

  packet[0] = PN532_PREAMBLE;
  packet[1] = PN532_STARTCODE1;
  packet[2] = PN532_STARTCODE2;
  packet[3] = LEN;
  packet[4] = ~LEN + 1;
  packet[5] = PN532_HOSTTOPN532;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < cmdlen; i++) {
    packet[6 + i] = cmd[i];
    sum += cmd[i];
  }
  packet[6 + cmdlen] = ~(PN532_HOSTTOPN532 + sum) + 1;
  packet[7 + cmdlen] = PN532_POSTAMBLE;

  int ret = i2c_write_blocking(I2C_PORT, NFC_ADDR, packet, 8 + cmdlen, false);
  if (ret != 8 + cmdlen) {
    printf("Error in writecommand function: %d\n", ret);
  }
}

/**************************************************************************/
/*!
    @brief  Return true if the PN532 is ready with a response.
*/
/**************************************************************************/
bool isready() {
  // I2C ready check via reading RDY byte
  uint8_t rdy[1];
  int ret = i2c_read_blocking(I2C_PORT, NFC_ADDR, rdy, 1, false);
  if (ret != 1) {
    printf("Error in isready function: %d\n", ret);
  }
  return rdy[0] == PN532_I2C_READY;
}

/**************************************************************************/
/*!
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool waitready(uint16_t timeout) {
  uint16_t timer = 0;
  while (!isready()) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
        return false;
      }
    }
    sleep_ms(10);
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads n bytes of data from the PN532 via SPI or I2C.

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
void readdata(uint8_t *buff, uint8_t n) {
  // I2C read
  uint8_t rbuff[n + 1]; // +1 for leading RDY byte
  int ret = i2c_read_blocking(I2C_PORT, NFC_ADDR, rbuff, n + 1, false);
  if (ret < 0) {
    printf("Error in readdata function: %d\n", ret);
  }
  for (uint8_t i = 0; i < n; i++) {
    buff[i] = rbuff[i + 1];
  }
}

/**************************************************************************/
/*!
    @brief  Tries to read the SPI or I2C ACK signal
*/
/**************************************************************************/
bool readack() {
  uint8_t ackbuff[6];
  readdata(ackbuff, 6);
  return (0 == memcmp(ackbuff, pn532ack, 6));
}

/**************************************************************************/
/*!
    @brief  Sends a command and waits a specified period for the ACK

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes
    @param  timeout   timeout before giving up

    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
bool sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {
  // I2C works without using IRQ pin by polling for RDY byte
  // seems to work best with some delays between transactions

  // write the command
  writecommand(cmd, cmdlen);

  // I2C TUNING
  sleep_ms(1);

  // Wait for chip to say it's ready!
  if (!waitready(timeout)) {
    return false;
  }

  // read acknowledgement
  if (!readack()) {
    printf("No ACK frame received!");
    return false;
  }

  // I2C TUNING
  sleep_ms(1);

  // Wait for chip to say it's ready!
  if (!waitready(timeout)) {
    return false;
  }

  return true; // ack'd command
}

/**************************************************************************/
/*!
    Reads the ID of the passive target the reader has deteceted.

    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool readDetectedPassiveTargetID(uint8_t *uid, uint8_t *uidLength) {
  // read data packet
  readdata(pn532_packetbuffer, 20);
  // check some basic stuff

  /* ISO14443A card response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */

  if (pn532_packetbuffer[7] != 1)
    return 0;

  uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];

  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];
  for (uint8_t i = 0; i < pn532_packetbuffer[12]; i++) {
    uid[i] = pn532_packetbuffer[13 + i];
  }

  return 1;
}

/**************************************************************************/
/*!
    @brief   Waits for an ISO14443A target to enter the field and reads
             its ID.

    @param   cardbaudrate  Baud rate of the card
    @param   uid           Pointer to the array that will be populated
                           with the card's UID (up to 7 bytes)
    @param   uidLength     Pointer to the variable that will hold the
                           length of the card's UID.
    @param   timeout       Timeout in milliseconds.

    @return  1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid, uint8_t *uidLength,
                         uint16_t timeout) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1; // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;

  if (!sendCommandCheckAck(pn532_packetbuffer, 3, timeout)) {
    return 0x0; // no cards read
  }

  return readDetectedPassiveTargetID(uid, uidLength);
}

/**************************************************************************/
/*!
    @brief   Configures the SAM (Secure Access Module)
    @return  true on success, false otherwise.
*/
/**************************************************************************/
bool SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (!sendCommandCheckAck(pn532_packetbuffer, 4, 1000))
    return false;

  // read data packet
  readdata(pn532_packetbuffer, 9);

  int offset = 6;
  return (pn532_packetbuffer[offset] == 0x15);
}

int main() {
  stdio_init_all();
  time_init();

  // Wait for the USB Debug port
  //      while (!stdio_usb_connected()) tight_loop_contents();
  sleep_ms(3000);

  printf("Starting\n");

  sd_card_t *pSD = sd_get_by_num(0);

  gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

  pwm_config config = pwm_get_default_config();
  // Set the PWM frequency (50 Hz is common for servos)
  pwm_config_set_clkdiv(&config, 100.0f);
  uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
  pwm_init(slice_num, &config, true);

  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  SAMConfig();

  uint8_t lastUid[] = {0, 0, 0, 0, 0, 0, 0};

  while (1) {
    pwm_set_gpio_level(SERVO_PIN, 0);

    uint8_t success;
    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
    uint8_t uidLength =
        0; // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

    // Wait for an NTAG203 card.  When one is found 'uid' will be populated with
    // the UID, and uidLength will indicate the size of the UUID (normally 7)
    success =
        readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 1000);

    if (success && memcmp(lastUid, uid, uidLength) != 0) {
      FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
      if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
      }
      FIL fil;
      fr = f_open(&fil, FILENAME, FA_OPEN_APPEND | FA_WRITE);
      if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", FILENAME, FRESULT_str(fr), fr);
      }

      printf("UID: ");
      for (int i = 0; i < uidLength; i++) {
        printf("%x", uid[i]);
        f_printf(&fil, "%x", uid[i]); // prints to the log file
      }
      printf("\n");
      f_printf(&fil, "\n");

      fr = f_close(&fil);
      if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
      }
      f_unmount(pSD->pcName);

      pwm_set_gpio_level(SERVO_PIN, 500);
      sleep_ms(500);
    }

    memcpy(lastUid, uid, 7);
  }
  return 0;
}
