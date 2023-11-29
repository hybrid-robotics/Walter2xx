#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLHC_Unified.h>
     
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
     
void setup(void) {
    Serial.begin(9600);
    Serial.println("Magnetometer Test");
    Serial.println("");

    /* Initialize the sensor */
    if (!mag.begin())  {
        /* There was a problem detecting the LSM303 Magnetometer (Compass) ... check your connections */
        Serial.println("Ooops, no LSM303 Magnetometer (Compass) detected ... Check your wiring!");
        while(1);
    }
}
     
void loop(void) {
    float heading, Pi = 3.14159;
    sensors_event_t event;

    /* Get a new sensor event */
    mag.getEvent(&event);

    // Calculate the angle of the vector y,x
    heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

    // Normalize to 0-360
    if (heading < 0) {
        heading = 360 + heading;
    }

    Serial.print("Compass Heading: ");
    Serial.println(heading);
    delay(500);
}