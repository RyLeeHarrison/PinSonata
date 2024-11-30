#include <PinSonata.hpp>

PinSonata pinManager;
PinSonataAnalog analogManager;
PinSonataInterrupt interruptManager;

float temperature = 0.0;
const float TEMP_THRESHOLD = 30.0; // Threshold in °C

// Event Handlers
void buttonPressed() {
    // Toggle the main LED
    pinManager.toggle("MAIN_LED");

    // Read temperature
    temperature = analogManager.read_voltage("TEMP_SENSOR");

    // Check for over-threshold condition
    if (temperature > TEMP_THRESHOLD) {
        Serial.println("Temperature exceeds threshold!");
        pinManager("WARNING_LED", HIGH); // Turn on warning LED
    }
}

void resetPressed() {
    // Reset system state
    pinManager("MAIN_LED", LOW);
    pinManager("WARNING_LED", LOW);
    Serial.println("System reset.");
}

void setup() {
    Serial.begin(9600);

    // Configure GPIO pins
    pinManager.configure_pins({
        {"MAIN_LED", 13},      // Main LED
        {"WARNING_LED", 14},   // Warning LED
        {"RESET_BUTTON", 12},  // Reset button
        {"TEMP_SENSOR", A0}    // Temperature sensor
    });
    pinManager.initialize_pins();

    // Attach interrupts
    interruptManager.configure_pins({
        {"TRIGGER_BUTTON", 11} // Button to trigger events
    });
    interruptManager.attach_interrupt(
        "TRIGGER_BUTTON",
        buttonPressed,
        PinSonataInterrupt::InterruptMode::EDGE_RISING
    );

    interruptManager.attach_interrupt(
        "RESET_BUTTON",
        resetPressed,
        PinSonataInterrupt::InterruptMode::EDGE_RISING
    );
}

void loop() {
    // Additional functionality could be added here
    delay(100);
}