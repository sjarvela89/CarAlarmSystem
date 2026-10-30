# Pinout

## Arduino Mega 2560

- `Serial1` = GSM/SIM900
- `Serial3` = GPS/NMEA

## Digital pins

- `D48` = PIR sensor output
- `D22` = immobilizer relay control
- `D23` = red status LED
- `D24` = green status LED

## Analog pins

- `A0` = battery voltage sense input through a resistor divider

## Notes

- The relay pin polarity is configurable in the sketch.
- Do not connect the vehicle battery directly to `A0`.
