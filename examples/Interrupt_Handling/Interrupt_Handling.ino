#include <PinSonata.hpp>

PinSonataInterrupt interruptManager;

void buttonPressed() {
    Serial.println("Button Pressed!");
}

void setup() {
    Serial.begin(9600);
    interruptManager.configure_pins({
        {"BUTTON", 12}
    });
    interruptManager.attach_interrupt("BUTTON", buttonPressed, PinSonataInterrupt::InterruptMode::EDGE_RISING);
}

void loop() {
    delay(1000); // Main loop remains available for other tasks
}
