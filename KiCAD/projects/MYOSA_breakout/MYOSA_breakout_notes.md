



**NOTES**



&#x09;**PinHeader\_1x13\_P2.54mm\_Vertical.step**

&#x09;

&#x09;Uni has KiCAD v9.01.

&#x09;

&#x09;CANT USE ADC2 WITH WIFI! Using 'sensor\_vp' as this is definitely ADC1\_CH0.	It doesn't have internal pulldown though so that's been added.	



&#x09;Pretty sure the onboard regulator will safely power the esp32 from VIN with	4.5-12V.



&#x09;Need high current for LED and 3V3 available for opamp circuit.



&#x09;Just use onboard 3V3 and external 5V? How about 4xAA at 6V?



&#x09;could 3v3 output on MYOSA power opamps? Yes, it doesn't really draw power.



&#x09;Want to make easy access for powering with PSU. Switch to choose between 	battery and PSU? If MYOSA is plugged in, still need 5V for LED.



&#x09;	battery: powering both MYOSA and LED

&#x09;	PSU: powering both MYOSA and LED

&#x09;	USB: Powers only MYOSA, battery/PSU required for LED



&#x09;Test points? 

&#x09;	

&#x09;	Voltage sources

&#x09;	PWM out

&#x09;	Either end of LED



**Measurements required/things to check**



&#x09;9V Battery holder

&#x09;LED Size

&#x09;Banana plugs

&#x09;Switches

&#x09;Connectors to BMP module/LED+Photodiode



&#x09;Should BMP180 be separate connector?

&#x09;Should battery be on other side?



**LED:** 1% duty cycle, 20us on, 1980us off (500Hz)



&#x09;3R4 series resistor drops 1.7V so LED has 3.3V across it, corresponding to 	\~450mA.



&#x09;P\_LED = 0.45 \* 3.3 \* 0.01 = 14.85mW



&#x09;P\_R\_SERIES = 0.45 \* 1.7 \* 0.01 = 7.65mW



**MOSFET**



&#x09;Switching freq = 500Hz

&#x09;Static drain-source on-resistance \~ 0.025R

&#x09;negligible power I think...



**MYOSA**



&#x09;Powered by 5V USB or 3.3Vin.



&#x09;Probably want to maintain 5V supply to LED so that there is a current 	limiting resistor and some headroom? Otherwise, could power with 3.3V and 	no resistor. Still need something that can supply the current spikes, MYOSA 	can't.



&#x09;ESP32 Max transmission power: 239mA (average) \* 3.3 = 789mW (Peak 379mA)

&#x09;lowest = 165mA \* 3.3 = 545mW.



has **AMS1117 (T33 F80LC)** fixed 3V3 voltage regulator.

&#x09;< 1A output

&#x09;< Absolute max 15V input (4.5-7V recommended)

&#x09;1.3V dropout at worst so 4.6V minimum



**ESP32-WROOM-32E**



&#x09;Absolute max input 3.6V. Provided 3V3 by AMS1117.



&#x09;All GPIO pins support either LED or motor PWM.

