#include <PinSonata.hpp>

PinSonataAnalog analogManager;

void setup() {
    Serial.begin(9600);
    analogManager.configure_pins({
        {"TEMP_SENSOR", 18} // Analog pin
    });
}

void loop() {
    float voltage = analogManager.read_voltage("TEMP_SENSOR");
    Serial.println(voltage);
    delay(1000);
}
