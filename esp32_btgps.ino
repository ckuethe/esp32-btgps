/*
usb/bluetooth gps. These things used to be common like weeds, and then we all
got cell phones with built in GPS.

This is built on a heltec wifi-kit-32 with ESP32+SSD1306 dev board, so that I
can use it as a standalone display, a USB gps dongle, or a bluetooth GPS for
a device that doesn't have a good onboard GPS.
*/

#include "Arduino.h"
#include "heltec.h"
#include "monospace.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "BluetoothSerial.h"

// Shouldn't be necessary. Heltec boards have bluetooth enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only availables for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
EspSoftwareSerial::UART gps_uart;
Adafruit_GPS GPS(&gps_uart);

#define LED_PIN 25
#define FIX_PIN 5
#define GTX_PIN 18
#define GRX_PIN 23

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "7399";  // Change this to more secure PIN.

// #define WAIT_FOR_BT // Uncomment this to wait for a bluetooth connection. Probably a bad idea

#define USBGPS  // Uncomment this to also act like a USB GPS

char *fixtype[] = { "none", " GPS", "DGPS", "?3?", "?4?", "?5?", "XXXX" };
int32_t last_display_time = millis();
int update_ms = 1000;

// Distance conversions, knots to...
float m_s = 0.514444, mph = 1.15078, kmh = 1.852;

#define DISPLAY_LINE_LEN 22
char lbuf[DISPLAY_LINE_LEN + 1];

void gps_setup() {
  // Do various setup things, eg. set dynamics mode, enable messages, ...
  char pubx00[] = {
    0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf1, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x30
  };
  
  char sbas_enable[] = {
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0xD5
  };

  char jam_detect[] = {
    0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45
  };

  char no_powersave[] = {
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91
  };

  // FIXME - compute these on the fly.
  char aero4g[] = { // 4g, 25m pmask
  //                                                  vv this byte is dynamics mode
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x19, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xD9
  };//                                                      ^^    ^^ 0x19 0x00 is accuracy mask
  char aero2g[] = { // 2g, 25m pmask
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x19, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xB7
  };
  char aero1g[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0xDB
  };
  char portable[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0F
  };

  gps_uart.write(pubx00, sizeof(pubx00));
  gps_uart.flush();
  delay(50);

  gps_uart.write(jam_detect, sizeof(jam_detect));
  gps_uart.flush();
  delay(50);

  gps_uart.write(no_powersave, sizeof(no_powersave));
  gps_uart.flush();
  delay(50);

//  gps_uart.write(dynamics4g, sizeof(dynamics4g));
//  gps_uart.write(dynamics2g, sizeof(dynamics2g));
//  gps_uart.write(dynamics1g, sizeof(dynamics1g));
  gps_uart.write(portable, sizeof(portable));
  gps_uart.flush();
  delay(50);

  gps_uart.write(sbas_enable, sizeof(sbas_enable));
  gps_uart.flush();
  delay(50);


}

void setup() {
  Heltec.begin(
    true /*Display */,
    false /*LoRa */,
    true /*Serial */);

  char device_name[24];
  snprintf(device_name, sizeof(device_name)-1, "btgps-%06llx", (ESP.getEfuseMac()>>24)&0xffffff);

  Heltec.display->setFont(DejaVu_Sans_Mono_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->clear();
  Heltec.display->drawString(0, 16, "GPS Receiver");
  Heltec.display->drawString(0, 32, device_name);
  Heltec.display->display();

  gps_uart.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, GRX_PIN, GTX_PIN, false, 95);
  GPS.begin(9600);
  gps_setup();

  SerialBT.begin(device_name);
  SerialBT.enableSSP();
#ifdef USE_PIN
  SerialBT.setPin(pin);
#endif

#ifdef WAIT_FOR_BT
  int is_bluetooth_available, is_bluetooth_ready, has_bluetooth_client;
  String waitmsg = "Waiting for Bluetooth";
  Serial.println(waitmsg);
  Heltec.display->drawString(0, 48, waitmsg);
  Heltec.display->display();

  do {
    delay(500);

    is_bluetooth_available = SerialBT.is_bluetooth_availableailable();
    has_bluetooth_client = SerialBT.hasClient();
    ir = SerialBT.isReady();
    Serial.printf("Bluetooth isAvailable %d\n", is_bluetooth_available);
    Serial.printf("Bluetooth hasClient %d\n", has_bluetooth_client);
    Serial.printf("Bluetooth isReady %d\n", is_bluetooth_ready);

    if (has_bluetooth_client || is_bluetooth_available)
      break;
  } while (1);
#endif  // WAIT_FOR_BT
} // setup()

void print_gps() {
  Serial.printf("\nDate: 20%02d/%02d/%02d %02d:%02d:%02d.%03d\n", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Fix: %s Quality: %dd %s\n", GPS.fix ? "OK" : "NO", GPS.fixquality_3d, fixtype[GPS.fixquality]);
  Serial.printf("Location: %f %f\n", GPS.latitudeDegrees, GPS.longitudeDegrees);
  Serial.printf("Speed: %.1fm/s Altitude: %.1fm Track: %.1f\n", GPS.speed * m_s, GPS.altitude, GPS.angle);
  Serial.printf("Satellites: %d\n", GPS.satellites);
}


void display_gps() {
  float area51[2] = { 37.23333, -115.80833 };
  int row = 0;
  Heltec.display->clear();
  snprintf(lbuf, DISPLAY_LINE_LEN,
    "20%02d/%02d/%02d  %02d:%02d:%02d",
    GPS.year, GPS.month, GPS.day,
    GPS.hour, GPS.minute, GPS.seconds);
  Heltec.display->drawString(0, row, lbuf);

  row = 16;
  snprintf(lbuf, DISPLAY_LINE_LEN, "%+9.5f %+10.5f", GPS.latitudeDegrees, GPS.longitudeDegrees);
  // snprintf(lbuf, DISPLAY_LINE_LEN, "%+9.5f %+10.5f", area51[0], area51[1]);
  Heltec.display->drawString(0, row, lbuf);

  row += 9;
  snprintf(lbuf, DISPLAY_LINE_LEN, "%5.1fm/s %+8.1fm", GPS.speed * m_s, GPS.altitude);
  Heltec.display->drawString(0, row, lbuf);

  row += 9;
  snprintf(lbuf, DISPLAY_LINE_LEN, "HDOP:%5.1f VDOP:%5.1f", GPS.HDOP, GPS.VDOP);
  Heltec.display->drawString(0, row, lbuf);

  row = 53;
  snprintf(lbuf, DISPLAY_LINE_LEN,
    "[%s %dD %4s %2d] %3d°",
    GPS.fix ? "OK" : "NO",
    GPS.fixquality_3d,
    fixtype[GPS.fixquality],
    GPS.satellites,
    (int)GPS.angle);
  Heltec.display->drawString(0, row, lbuf);

  Heltec.display->display();
}

void loop() {

  char c = GPS.read();

#ifdef USBGPS
  if (Serial.available()) // relay control signals
    gps_uart.write(Serial.read());
#endif // USBGPS

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (SerialBT.hasClient())
      SerialBT.print(GPS.lastNMEA());
#ifdef USBGPS
    Serial.print(GPS.lastNMEA());
#endif // USBGPS
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (abs((int)(millis() - last_display_time)) > update_ms) {
    last_display_time = millis();  // reset the timer

    display_gps();
    //print_gps();
  }
}
