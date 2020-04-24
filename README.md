# Evaluation_Kits
Code used in our Air Quality Sensor Evaluation Kits

The AAS-AQS-UNO-RH-CO2 kit includes an SM-PWM-01C dust sensor, with a 0.96" OLED screen, and optional T6713 CO2 sensor, T9602 Humidity & Temperature Sensor.

The AAS-LDS-UNO-RH-CO2 kit includes an SM-UART-04L dust sensor, with a 0.96" OLED screen, and optional T6713 CO2 sensor, T9602 Humidity & Temperature Sensor.

Hardware is an Arduino Uno compatable with Amphenol Advanced Sensors Shield and 0.96" OLED SPI Screen; the Shield is not available seperately.

Code checks for sensors (except dust 01C sensor) and displays only those values. A Firmware 'switch' is available to change the output to a serial output for data logging in the AQS version, the LDS version displays and prints to port simultaneously.

NOTE: Any option with the SM-PWM-01C dust sensor requires the pinchangeint library to be installed. https://github.com/NicoHood/PinChangeInterrupt Or http://playground.arduino.cc/Main/PinChangeInt
