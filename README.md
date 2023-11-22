# MAINSolar
Solar Tracker

This progam is the main program for controlling a solar tracking solar panel. It has two modes: GPS and LDR 

BOM
- Chip: Development Boards & Kits - ARM MSP432 401R - MSP432P401R LaunchPad
- GPS: Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3
- IDE: Code Composer
- Solar Panel: RNG-50D
- Motors: 1500N Electric Linear Actuator 150mm Stroke 12v DC Motor Linear Motion Controller with Limit Switch JS-TGZ-U2
- Photo resistor array - used 4 photo resisitors aranged in a 3d printed square. Has high walls in the center so that shadows can be casted when pointing up. No model avaliable 

Most of the logic is in the main.c file

/************************************************************************
  Dual Axis Solar Tracking Controller for solar panels with GPS and LDR
  Last updated 12-11-2020
  Developed and written by:
      Danielle Cranston
      Mary Caroline Dowd
      Stephen Thelusme
      Andre Sands



  TASK SUMMARY
Choose mode based on switch
    GPS - Global Positioning
        Import GPS values
        put values in new array
        parse GPS VARIBLES
        calculate Azimuth, Zenith angles - finds suns position and represents in angles
        read in potentiometer - calculate panel position
        Calculate motor movement - finds time to move the motor
        move motors
        check threshold - Go back up to Sensor comparison if doesn't reach threshold or number of tries
    LDR - Light Dependent Resistors
L       Light sensor setup
        sensor comparison
        move motors
        check threshold - Go back up to Sensor comparison if doesn't reach threshold or number of times
   Sleep mode
       turn off all pins
       set up RTC - Real time Clock
       Goes to LPM3
       wake up in 20 mins

Devices:
    MSP432P401
    Adafruit Ultimate GPS
    L298N DC motor driver
    4 LDR
    4 2K  Ohm resistors\
    LM2596
    2 POT
    RNG-50D Renogy Solar panel

PIN---------Function
P5.5  |<--- A0 (Analog Input) LDR 0, NE              sensorBuffer[0]
P5.4  |<--- A1 (Analog Input) LDR 1, NW              sensorBuffer[1]
P5.2  |<--- A3 (Analog Input) LDR 3, SE              sensorBuffer[3]
P5.1  |<--- A4 (Analog Input) Potentiometer NS       sensorBuffer[4]
P5.0  |<--- A5 (Analog Input) Potentiometer EW       sensorBuffer[5]
P4.7  |<----A6 (Analog Input) LDR 2, SW              sensorBuffer[2]

P3.2  |<----RX(TX from device)
P3.3  |<----TX(RX from device)
P4.1  |<----GPS enable

P1.2  |<----UART:pc (not a pin, USB cord)
P1.3  |<----UART:pc (not a pin, USB cord)
P2.0  |<----RedLED
P2.1  |<----GreenLED
P2.2  |<----BlueLED

P3.5  |<----Mode toggle switch
P3.6  |<----Sensor enable
P3.7  |<----Pot enable

POTNS
    G- MC
    BR- GND
    BL- VOLTAGE


POTEW
    BL - POWER
    G - MC
    BR - GND

IN 1 ----P2.7--------North-----NS+
IN 2 ----P2.6--------South-----NS-
IN 3 ----P2.4--------East------EW+
IN 4 ----P5.6--------West------EW-
++ contract motor
+- extend motor




 ****************************************************************************/
/*
