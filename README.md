# telenorma_clock_arduino
An arduino controller for an telenorma clock using an RTC DS1302 and a 16x2 display with buttons shield.

The board used is an Arduino Uno

The clock is controlled by 2 12V relays connected to the PC1 and PC2. The RTC is connected to P3 to p5. 
The clock is detecting loss of power and saves the current phisical clock position on EEPROM. This detection is done on PD2 (INT0).
