# AVR-Sensor-Alarm-with-Lock
AVR ATMEGA 16 CO sensor system that collects CO values translated into Voltage, converts it via internal ADC and displays the danger level on LCD and LEDs. There is also a lock system that is managed by keyboard password.

### Parts that were used
* CO sensor: ULPSM-CO 968-001
* LCD: HD44780
* KEYBOARD: Generic 4x4 keyboard
* Internal 10 bit Analog to Digital Converter (ADC)

### How it works
- Measure CO value via ADC every 100 ms
- Show CO level on LEDS (More details below)
- If CO level>70ppm LCD shows "GAS DETECTED" and level leds blink (ALARM IS ON).
- Otherwise "CLEAR" is displayed and leds are not flashing
- Regarding the password aspect:
  - Correct password while alarm is on 




