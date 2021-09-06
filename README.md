# LMP91000
Arduino library for the LMP91000 AFE Potentiostat for Electrochemical Sensing

Forked from [LinnesLab repo](https://github.com/LinnesLab/LMP91000)

LMP91000 connections / Arduino Connections

DGND / GND  
MENB / Arduino Digital Pin  
SCL / SCL (with 10k pull-up resistor)  
SDA / SDA (with 10k pull-up resistor)  
VDD / 3.3V  
AGND / GND  
CE / Counter electrode  
RE / Reference electrode  
WE / Working electrode  
VREF / Using an externla precision LM4040 voltage reference (4.1 V)  
C1 / not needed  
C2 / not needed / Pin Connected to Digital Analog Pin for secondary read  
Vout / Analog Input A0  
