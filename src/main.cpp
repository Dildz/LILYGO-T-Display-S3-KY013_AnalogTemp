/*********************************************************************************************************
 * LILYGO T-Display-S3 KY-013 Thermistor Temperature Sensor Project
 *
 * Description:
 *   This code reads temperature data from a KY-013 thermistor sensor and displays it on the built-in
 *   screen of the LilyGO T-Display-S3 using the TFT_eSPI library.
 *   The code uses state machine logic to avoid using delays, which is code blocking.
 *   The temperature is displayed in both Celsius and Fahrenheit.
 *   The screen is only updated if there is a change in the temperature readings.
 *
 * How It Works:
 *   1. Sensor Reading: The code reads analog data from the KY-013 sensor at regular 2-second intervals.
 *   2. Converts to resistance using series resistor value.
 *   3. Calculates temperature using Steinhart-Hart equation & applies a calibration offset.
 *   2. Display: The temperature data is updated on the screen only when there is a difference in readings.
 *   3. State Machine: A state machine is used to manage the timing of sensor readings and display updates.
 *
 * Pin Connections:
 *   - KY-013 Signal Pin -> GPIO1 (ADC1_CH0)
 *   - LCD Backlight     -> GPIO15
 *   - Ground            -> GND
 *   - Voltage           -> 3.3V
 *
 * Notes:
 *   - Default uses Steinhart-Hart coefficients for generic 10K thermistor.
 *   - CALIBRATION_OFFSET can be adjusted to calibrate the sensor.
 *   - For best accuracy:
 *      - calibrate against known reference at multiple temperatures (e.g. DS18B20, multimeter, body temp),
 *      - use an average offset that will work across your expected temperature range.
 *   - The sensor's resistance is inverted, so we account for this by dividing by (ADC_MAX - analogValue).
 *   - The TFT_eSPI library is configured to work with the LilyGO T-Display-S3.
 *
 * KY-013 Specifications:
 *   - Operating Voltage: 3.3V to 5V
 *   - Temperature Range: -55°C to 125°C
 *   - Accuracy: ±1°C
 * 
 **********************************************************************************************************/

/*************************************************************
******************* INCLUDES & DEFINITIONS *******************
**************************************************************/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>

// TFT_eSPI
TFT_eSPI tft = TFT_eSPI();

// KY-013 Thermistor Sensor
#define THERMISTOR_PIN 1         // analog pin connected to the KY-013 (ADC1_CH0)
#define R1 10000.0               // value of series resistor
#define ADC_MAX 4095.0           // 12-bit ADC resolution
#define CALIBRATION_OFFSET -0.8f // rough offset using DS18B20 with 2 points - ambient & body temperature

// Steinhart-Hart coefficients from example
#define C1 0.001129148
#define C2 0.000234125
#define C3 0.0000000876741

// Text positions for dynamic updates
#define CELSIUS_Y 85
#define FAHRENHEIT_Y 133
#define ERROR_Y 85

// State Machine States
enum class State {
  READ_SENSOR,    // state for reading sensor data
  UPDATE_DISPLAY, // state for updating the display
  WAIT            // state for waiting between sensor reads
};

// Global variables
State currentState = State::READ_SENSOR;       // initial state
unsigned long previousMillis = 0;              // for non-blocking timing
const unsigned long sensorReadInterval = 2000; // read sensor every 2 seconds
float temperatureC = 0.0;                      // variable to store temperature in Celsius
float temperatureF = 0.0;                      // variable to store temperature in Fahrenheit
float previousTemperatureC = -100.0;           // store previous temperature value (init to impossible value)
bool sensorConnected = true;                   // flag to track sensor connection
bool valueChanged = false;                     // flag to indicate when values have changed

/*************************************************************
********************** HELPER FUNCTIONS **********************
**************************************************************/

// Function to draw the static screen elements
void drawStaticScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.println("---------------------------");
  tft.println("   KY013 Temp Sensor");
  tft.println("---------------------------");
  
  if (sensorConnected) {
    tft.println("\nTemp in Celcius:");
    tft.println("\n\nTemp in Fahrenheit:");
  }
}

// Function to show sensor error message
void showSensorError() {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, ERROR_Y);
  tft.println("\n!! Sensor Not Connected !!");
}

// Function to convert analog reading to temperature in Celsius
float readTemperatureCelsius(int analogValue) {
  // Check for disconnected sensor
  if (analogValue <= 1 || analogValue >= ADC_MAX-1) {
    return NAN;
  }
  
  // Calculate thermistor resistance
  float R2 = R1 * (float)analogValue / (ADC_MAX - (float)analogValue); // inverted resistance
  float logR2 = log(R2);
  
  // Steinhart-Hart equation
  float tempK = 1.0 / (C1 + C2*logR2 + C3*logR2*logR2*logR2); // temperature in Kelvin
  float offsetTemp = tempK - 273.15; // convert Kelvin to Celsius
  
  // Apply calibration offset
  offsetTemp += CALIBRATION_OFFSET;
  
  return offsetTemp;
}

// Function to convert Celsius to Fahrenheit
float celsiusToFahrenheit(float celsius) {
  return (celsius * 1.8) + 32.0;
}

// Function to update temperature values on screen
void updateTemperatureValues() {
  // Update Celsius value
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, CELSIUS_Y);
  tft.print(String(temperatureC, 1) + " °C   "); // 1 decimal place
  
  // Update Fahrenheit value
  tft.setCursor(0, FAHRENHEIT_Y);
  tft.print(String(temperatureF, 2) + " °F   "); // 2 decimal places
}

/*************************************************************
*********************** MAIN FUNCTIONS ***********************
**************************************************************/

// SETUP
void setup() {
  // Initialize the TFT display
  tft.init();
  tft.setRotation(0);                     // adjust rotation (0 & 2 portrait | 1 & 3 landscape)
  tft.fillScreen(TFT_BLACK);              // clear screen
  tft.setTextFont(2);                     // set the font
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // set text colour

  tft.println("Initialising...\n");
  delay(1000);

  // Set ADC settings
  analogReadResolution(12);       // set to 12-bit resolution
  analogSetAttenuation(ADC_11db); // for full 0-3.3V range
  delay(1000);

  // Draw the initial static screen
  drawStaticScreen();
}

// MAIN LOOP
void loop() {
  unsigned long currentMillis = millis(); // get the current millis time

  // State Machine Logic
  switch (currentState) {
    case State::READ_SENSOR: {
        // Read sensor data
        int analogValue = analogRead(THERMISTOR_PIN);
        temperatureC = readTemperatureCelsius(analogValue);
        
        // Check sensor connection
        bool currentConnectionState = !isnan(temperatureC);
        if (currentConnectionState != sensorConnected) {
          sensorConnected = currentConnectionState;
          drawStaticScreen();
          if (!sensorConnected) {
            showSensorError();
          }
        }
        
        // Only proceed if sensor is connected
        if (sensorConnected) {
          temperatureF = celsiusToFahrenheit(temperatureC);
          
          // Check if temperature changed significantly
          if (abs(temperatureC - previousTemperatureC) >= 0.1) {
            valueChanged = true;
            previousTemperatureC = temperatureC;
          }
        }
        
        currentState = State::UPDATE_DISPLAY;
        break;
      }
      

    case State::UPDATE_DISPLAY:
      // Update display only if values changed and sensor is connected
      if (valueChanged && sensorConnected) {
        updateTemperatureValues();
        valueChanged = false;
      }
      
      currentState = State::WAIT;
      previousMillis = currentMillis;
      break;

    case State::WAIT:
      // Wait for the specified interval before next reading
      if (currentMillis - previousMillis >= sensorReadInterval) {
        currentState = State::READ_SENSOR;
      }
      break;

    default:
      currentState = State::READ_SENSOR;
      break;
  }
}
