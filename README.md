# AVR-Sensor-Alarm-with-Lock
Microcontroller AVR ATMEGA 16 CO(Carbon Monoxide) sensor system that collects CO values translated into Voltage, converts it via internal ADC(Analog to Digital Converter) and displays the danger level on LCD and LEDs. There is also a lock system that is managed by keyboard password.

### Parts that were used
* CO sensor: ULPSM-CO 968-001
* LCD: HD44780
* KEYBOARD: Generic 4x4 keyboard
* Internal 10 bit Analog to Digital Converter (ADC)

### How it works
- Measure CO value via ADC every 100 ms (Timer Interrupts)
- Show CO level on LEDS (More details below)
- If CO level>70ppm LCD shows "GAS DETECTED" and level leds blink (ALARM IS ON).
- Otherwise "CLEAR" is displayed and leds are not flashing
- Regarding the password aspect:
  - Correct password while alarm is on leads to lcd "WELCOME" message and pause of the alarm for 4 seconds (LED PB7 IS ON)
  - Correct password while alarm is off leads to lcd "WELCOME" message and LED PB7 ON.
  - Wrong password in either case leads to "secondary" alarm LED PB7 to blink for 4 seconds.
  
## More Technical Aspects

#### How the conversion from CO value to digital binary was made
- The following conditions were considered to be true at all times: 
  - Vgas0=0.1 Volts (see datasheet)
  - Sensitivity Code=129 nA/ppm
  - Temperature 20 &deg; C 
  - Humidity 40%
 
 Taking the above into account and the fact that the internal ADC follows the linear conversion: Vgas=ADC* 5/1024
 we can calculate from Cx=1/M (Vgas-Vgas0) (see datasheet of CO sensor) the correspondence between the ppm and ADC value.
 
![antistoixia timwn CX ME ADC VAL](https://user-images.githubusercontent.com/56197365/103309168-072f9d00-4a1d-11eb-8f1e-8b7a8d36d770.png)

 
 #### Counting time and getting the correct ADC value
 ###### Analog to Digital Conversion
 * C: In C language we implemented a polling interrupt method. This means that we use the timer 1 interrupt every 100 ms and inside the IRS (interrupt service routine) we ask for a value from the ADC and wait until that conversion is done to get the correct value.
 * Assembly: In assembly language we take advantage of the AUTO-TRIGGER setting the ADC has. We set the ADC to be triggered by the 100 ms timer completion. After the completion of one analog to digital conversion , the interrupt service routine of the ADC is called.
###### Wait times for LEDs
 * One way is to use a second timer and interrupts. However, since the system does not need to be doing anything else at the time, we chose to freeze the system with functions that use NOPs to implement delay.
 
 ### Below you can see the flow diagram for the code.
 ###### Note: LCD display is not implemented in C.
![assembly flow diag 1_NEW-1](https://user-images.githubusercontent.com/56197365/103312450-c2f4ca80-4a25-11eb-8361-6632bfcef489.png)
![assembly flow diag 2-1](https://user-images.githubusercontent.com/56197365/103312458-c8521500-4a25-11eb-899d-374d97634a13.png)

 
 ## AUTHORS
 - *Dimitrios Lampros*
 - *Marios Mitropoulos*

 



