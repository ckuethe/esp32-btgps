# ESP32 BT GPS

Once upon a time, external bluetooth GPS receivers used to be A Thing.
These days, they're no longer cheap as chips, nor are they particularly
small.

As my dad would say "I could build that out of plywood!" So here we are.
Using a 
[Heltec wifi-kit-32](https://heltec.org/project/wifi-kit-32/)
(ESP32 + SSD1306 OLED) and a no-name
[Ublox NEO-6M](https://www.u-blox.com/en/product/neo-6-series)
breakout board I built one. The OLED display provides visual indication
of fix status, and the build takes advantage of the lithium charger
onboard for wireless operation. The firmware also allows the device to
be used as a generic usb GPS.

A couple of handy features:
* the device will pick a device name based on its mac address
* `$PUBX,00` fix messages are enabled to improve compatibility with
[Bluetooth GNSS](https://github.com/ykasidit/bluetooth_gnss/)
* Airborne <2g dynamics mode is enabled to improve operation in flight

## TODO
* relay ubx mode
* wifi captive portal
* wifi relay mode
