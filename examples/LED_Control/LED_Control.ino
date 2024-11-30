#include <PinSonata.hpp>

PinSonata pinManager;

void setup() {
    pinManager.configure_pins({
     // { "Key", GPIO }
        { "LED", 13 }
    });

    pinManager.initialize_pins();
}

void loop() {
    pinManager.toggle("LED"); // Toggle the LED state
    delay(500);               // Wait for 500ms
}
