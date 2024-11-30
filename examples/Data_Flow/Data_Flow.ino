#include <PinSonata.hpp>

PinSonataAnalog analogManager;
PinSonataInterrupt interruptManager;

float temperature = 0.0;

void readSensor() {
    temperature = analogManager.read_voltage("TEMP_SENSOR");
    Serial.print("Temperature: ");
    Serial.println(temperature);
}

void setup() {
    Serial.begin(9600);
    analogManager.configure_pins({
        {"TEMP_SENSOR", 34}
    });
    interruptManager.configure_pins({
        {"TRIGGER", 12}
    });
    interruptManager.attach_interrupt("TRIGGER", readSensor, PinSonataInterrupt::InterruptMode::EDGE_RISING);
}

void loop() {
    // Continues to operate independently of the interrupt
    delay(1000);
}
