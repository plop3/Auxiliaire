## Compilation et upload par OTA
  * arduino  --verify --preserve-temp-files --board esp8266:esp8266:d1_mini Auxiliaire.ino
  * espota -i auxiliaire.local -p 8266 -f /tmp/arduino_build.../Auxiliaire.ino.bin
