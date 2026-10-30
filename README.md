# Car Alarm System for Arduino Mega 2560

This project turns the thesis requirements into a working Arduino Mega sketch for a car alarm / anti-theft prototype.

## What is implemented

- SMS-based arming and disarming
- Immobilizer relay control
- PIR motion detection
- GPS location reply through SMS
- Battery voltage monitoring
- Status SMS
- Outgoing alarm call support

## Why this version is more reliable than the thesis prototype

The thesis describes the same target functions: alarm notification, setup/activation, immobilization, PIR reporting, GPS location by SMS, and low-battery detection. It also notes the prototype problems: SIM900 could not reliably provide caller number handling, and the `NO CARRIER` issue blocked the call-based flow. This project uses SMS commands as the control interface and a non-blocking state machine instead of the broken prototype loop.

## Commands

Send these SMS commands from the owner number:

- `ACTIVATE` or `ARM`
- `DEACTIVATE` or `DISARM`
- `START`
- `STOP`
- `LOCATION`
- `STATUS`
- `HANGUP`

## Wiring

- PIR sensor output -> Mega pin 48
- Immobilizer relay -> Mega pin 22
- Red LED -> Mega pin 23
- Green LED -> Mega pin 24
- Battery sense -> Mega A0 via a divider
- SIM900 GSM -> Mega Serial1
- GPS NMEA module -> Mega Serial3

## Important setup notes

1. Replace `OWNER_NUMBER` in `CarAlarmSystem.ino`.
2. Adjust `BATTERY_DIVIDER_RATIO` to match your resistor divider.
3. Invert `RELAY_ACTIVE_LEVEL` if your relay board is active LOW.
4. Confirm the SIM900 baud rate used by your module.
5. Test the GSM module outside the car before installation.

## Files

- `CarAlarmSystem.ino` - main sketch
