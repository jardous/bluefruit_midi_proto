/*
 *  https://developer.apple.com/bluetooth/Apple-Bluetooth-Low-Energy-MIDI-Specification.pdf
 *  
 *  HW connection:
 *  
 *  | Pro Mini | Bluefruit UART |
 *  +----------+----------------+
 *  |   GND    |      GND       |
 *  |   VCC    |      VIN       |
 *  |    11    |      TX0       |
 *  |    12    |      RX1       |
 *  |    13    |      CTS       |
 *  |     9    |      RST       |
 */

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


// Create the bluefruit object
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

#define TXRX_BUF_LEN                    20
#define RX_BUF_LEN                      100 // Overwriting RX Buf Len since we handle fragmentation
static int midi_rx_buf_num, rx_state = 0;
static uint8_t midi_rx_buf[RX_BUF_LEN];


int BUFLEN = 20;
byte outBuf[3];
char buf[20];
uint8_t size=3;

void wait_for_connection(void) {
    // Wait for connection
  while (ble.isConnected() == 0) {
    delay(500);
  }
}

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(9600);

  // Initialise the module
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  // Perform a factory reset to make sure everything is in a known state
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  // Print Bluefruit information
  ble.info();

  // configure GATT services and advertising
  ble.sendCommandCheckOK("AT+GATTLIST");
  ble.sendCommandCheckOK("AT+GATTADDSERVICE=UUID128=03-B8-0E-5A-ED-E8-4B-33-A7-51-6C-E3-4E-C4-C7-00");
  ble.sendCommandCheckOK("AT+GATTADDCHAR=UUID128=77-72-E5-DB-38-68-41-12-A1-A9-F2-66-9D-10-6B-F3,PROPERTIES=0x96,MIN_LEN=1,MAX_LEN=20");
  ble.sendCommandCheckOK("AT+GAPINTERVALS=8,15,250,180");
  ble.sendCommandCheckOK("AT+GAPSETADVDATA=02-01-06-11-06-00-C7-C4-4E-E3-6C-51-A7-33-4B-E8-ED-5A-0E-B8-03");
  ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=4");
  ble.reset();

  ble.verbose(false);  // debug info is a little annoying after this point!

  wait_for_connection();
}

unsigned char noteON = 0x90; // dec 144 = 10010000 in binary, note on command
unsigned char noteOFF = 0x80; // dec 128 = 10000000 in binary, note off command
unsigned char velocity = 100;

/*
 * Set the new characteristic value
 * format: AT+GATTCHAR=1,AF-80-XX-YY-ZZ
 * where XX is the command, YY is note and ZZ is velocity
 */
void ble_midi_send(unsigned char command, unsigned char param1, unsigned char param2) {
    static char *buf = "AT+GATTCHAR=1,AF-80-00-00-00";
    sprintf(buf+20, "%X-%02X-%02X", command, param1, param2);
    ble.sendCommandCheckOK(buf);
    Serial.println(buf);
}


void loop(void)
{
  for (int note=50; note<70; note++) {  // from note 50 (D3) to note 69 (A4)
    ble_midi_send(noteON, note, velocity);  //turn note on
    delay(300);  // hold note for 300ms

    ble_midi_send(noteOFF, note, 0);
    delay(200);  // wait 200ms until triggering next note

    wait_for_connection();
  }
}
