LILYGO T-Display-S3 KY-013 Temperature Sensor Module Project

This code reads the analog signal from the KY-013 module and displays the temperature on the built-in screen of the LilyGO T-Display-S3 using the TFT_eSPI library. The Steinhart-Hart equation is used to convert the sensor reading into Kelvin & then into Celsius. The screen is only updated if there is a change in the sensor reading.

Pin Connections:
 - KY-013 VCC  -> 3.3V
 - KY-013 GND  -> GND
 - KY-013 S    -> GPIO1

KY-013 Specifications:
 - Operating Voltage: 3.3V to 5V
 - Temperature Range: -55°C to 125°C
 - Accuracy: ±1°C
