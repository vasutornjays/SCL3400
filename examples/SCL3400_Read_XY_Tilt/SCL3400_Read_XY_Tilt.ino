/* Read Data from Murata SCL3400 Inclinometer
 * Version 0.1.0 - September 13, 2024
 * SCL3400_Read_XY_Tilt
*/

#include <SPI.h>
#include <SCL3400.h>

SCL3400 inclinometer;
//Default SPI chip/slave select pin is D10

// Need the following define for SAMD processors
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

void setup() {
  Serial.begin(115200);
  delay(1000); //SAMD boards may need a long time to init SerialUSB
  Serial.println("Reading basic Tilt Level Offset values from SCL3400 Inclinometer");

  if (inclinometer.begin() == false) {
    Serial.println("Murata SCL3400 inclinometer not connected.");
    while(1); //Freeze
  }
  Serial.println("Murata SCL3400 inclinometer connected.");

  inclinometer.setMode(SCL3400_MODE_B); // Set the mode to A (-90, 90 degrees)

}

void loop() {
  if (inclinometer.available()) { //Get next block of data from sensor
    Serial.print("X_acc:");
    Serial.print(inclinometer.getCalculatedAccelerometerX(), 6);
    Serial.print(",");
    Serial.print("X_Tilt:");
    Serial.print(inclinometer.getcalculateTiltX(), 6);
    Serial.print(",");
    Serial.print("Y_acc:");
    Serial.print(inclinometer.getCalculatedAccelerometerY(), 6);
    Serial.print(",");
    Serial.print("Y_Tilt:");
    Serial.print(inclinometer.getcalculateTiltY(), 6);
    Serial.print(",");
    Serial.print("Temp:");
    Serial.println(inclinometer.getCalculatedTemperatureCelsius(), 6);
    delay(10); //Allow a little time to see the output
  } else inclinometer.reset();
  // Serial.println("Loop");
  delay(10);
}
