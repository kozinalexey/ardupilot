I tried to maintain compatibility with the OpenPilot documentation. The main difference - FlexiPort is 
always Serial and external I2C port is on pins 7&8 of Input port.


Main Port - telemetry, Serial1. As a variant it can be used as SBUS input with hardware inverter (requires recompilation)
FlexiPort - OSD, Serial2


Input Port - PWM input is not supported - this is general trend
pin 1 of Input port is GND
pin 2 of Input port is +5
pin 3 of Input port is PPM/SBUS (SBUS works, DSM still not tested)
pin 4 of Input port is a Buzzer pin (requires additional transistor!)
pins 5&6 of Input port are Tx and Rx of Serial3 (for GPS)
pins 7&8 of Input port are SCL and SDA of external I2C


Output Port for MOTORs
Connect to PWM output pins in ArduCopter order

5&6 PWM Output pins are Rx and Tx of Serial4 - but only for quads

PWM input is not supported - this is general trend



OpLink port

DSM satellite can be connected to Oplink port (hardware Serial5) or to PPM input (pin 3 of input port) via Software Serial (DSM still not tested yet)

Support for binding of DSM satellite requires some additional hardware - managed stabilizer 3.3 volts. 
So it will be best to use satellites with Bind button.

Connection to OpLink port
Pin 1 is Gnd, 
pin 2 is +5(DSM sat requires 3.3!)
pin 3 is Rx 
pin 4 is Enable for 3.3 stab. 

So it will be best to use satellites with Bind button.

