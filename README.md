# PinSonata 🚀

**PinSonata** is a highly flexible Arduino library that orchestrates GPIO pins, analog readings, and interrupts into a seamless "sonata" of data flows. From simple LED control to interfacing advanced Analog-to-Digital Converters (ADCs), PinSonata enables developers to create complex and efficient workflows with ease. (Used ChatGPT to write this README, so much faster...).
---

## Features

1. **Chained Data Flows**  
   Combine GPIO control, analog readings, and interrupts into a cohesive data flow for dynamic control systems.

2. **Cross-Platform Compatibility**  
   Designed to work seamlessly with ESP32, RP2040 (Raspberry Pi Pico), and standard Arduino platforms. Automatically adapts to platform-specific features.

3. **Versatile Pin Management**  
   Manage GPIO pins with descriptive names. Simplify tasks like toggling, pulsing, and waiting for pin state changes.

4. **Advanced Analog Operations**  
   Perform voltage conversions, multi-sample averaging, and high-resolution ADC readings.

5. **Interrupt Handling**  
   Attach interrupts with customizable modes to handle real-time pin state changes or event triggers.

---

## API Breakdown

The **PinSonata** library is divided into three main classes:
1. **PinSonata**: Core GPIO pin management.
2. **PinSonataAnalog**: Extension for handling analog pins and ADC operations.
3. **PinSonataInterrupt**: Extension for managing interrupts on GPIO pins.

### 1. **PinSonata Class**
This is the core class for managing GPIO pins.

#### Functions
- **`configure_pins(std::map<std::string, int> pinMap)`**  
  Maps descriptive names to GPIO pin numbers.  
  Example:  
  ```cpp
  pinManager.configure_pins({{"LED", 13}, {"BUTTON", 12}});
  ```

- **`initialize_pins()`**  
  Sets all configured pins to `OUTPUT` mode and initializes their state to `LOW`.  
  Example:  
  ```cpp
  pinManager.initialize_pins();
  ```

- **`operator()(std::string pinName, int state)`**  
  Sets the state (`HIGH` or `LOW`) of a pin using its descriptive name.  
  Example:  
  ```cpp
  pinManager("LED", HIGH); // Turn the LED on
  ```

- **`read(std::string pinName)`**  
  Reads the current digital state (`HIGH` or `LOW`) of a pin.  
  Example:  
  ```cpp
  int state = pinManager.read("BUTTON");
  ```

- **`toggle(std::string pinName)`**  
  Toggles the current state of a pin.  
  Example:  
  ```cpp
  pinManager.toggle("LED"); // Switch LED state
  ```

- **`pulse(std::string pinName, unsigned long duration)`**  
  Sets a pin `HIGH` for a specified duration (in microseconds) and then back to `LOW`.  
  Example:  
  ```cpp
  pinManager.pulse("LED", 100000); // Pulse for 100ms
  ```

- **`wait_for_state(std::string pinName, int desiredState, unsigned long timeout_ms = 1000)`**  
  Waits until a pin reaches the desired state within a timeout. Returns `true` if successful, `false` otherwise.  
  Example:  
  ```cpp
  if (pinManager.wait_for_state("BUTTON", HIGH, 5000)) {
      Serial.println("Button pressed!");
  }
  ```

- **`wait_for_change(std::string pinName, unsigned long timeout_ms = 1000)`**  
  Waits for a pin’s state to change within a timeout. Returns `true` if successful, `false` otherwise.  
  Example:  
  ```cpp
  if (pinManager.wait_for_change("BUTTON", 3000)) {
      Serial.println("Button state changed!");
  }
  ```

- **`has_pin(std::string pinName)`**  
  Checks if a pin name is mapped.  
  Example:  
  ```cpp
  if (pinManager.has_pin("LED")) {
      Serial.println("LED is configured!");
  }
  ```

- **`get_gpio(std::string pinName)`**  
  Retrieves the GPIO number for a pin name. Returns `-1` if the pin is not mapped.  
  Example:  
  ```cpp
  int gpio = pinManager.get_gpio("LED");
  ```

---

### 2. **PinSonataAnalog Class**
Extends `PinSonata` to include analog operations like reading values and voltages.

#### Functions
- **`read_analog(std::string pinName)`**  
  Reads the raw analog value from the specified pin.  
  Example:  
  ```cpp
  int rawValue = analogManager.read_analog("TEMP_SENSOR");
  ```

- **`read_voltage(std::string pinName, float reference_voltage = PINSONATA_REFERENCE_VOLTAGE)`**  
  Converts the raw analog reading to a voltage.  
  Example:  
  ```cpp
  float voltage = analogManager.read_voltage("TEMP_SENSOR");
  ```

- **`read_analog_averaged(std::string pinName, uint8_t samples = 10, uint8_t delay_ms = 1)`**  
  Averages multiple analog readings to produce a more stable result.  
  Example:  
  ```cpp
  int averageValue = analogManager.read_analog_averaged("TEMP_SENSOR", 20, 5);
  ```

- **`set_adc_resolution(AdcResolution resolution)`** *(Platform-dependent)*  
  Sets the ADC resolution for analog readings (9, 10, 11, or 12 bits).  
  Example:  
  ```cpp
  analogManager.set_adc_resolution(PinSonataAnalog::AdcResolution::BITS_12);
  ```

---

### 3. **PinSonataInterrupt Class**
Extends `PinSonata` to manage GPIO interrupts.

#### Functions
- **`attach_interrupt(std::string pinName, void (*isr)(), InterruptMode mode)`**  
  Attaches an interrupt service routine (ISR) to a pin.  
  Example:  
  ```cpp
  interruptManager.attach_interrupt("BUTTON", buttonPressed, PinSonataInterrupt::InterruptMode::EDGE_RISING);
  ```

- **`detach_interrupt(std::string pinName)`**  
  Detaches the interrupt from a pin.  
  Example:  
  ```cpp
  interruptManager.detach_interrupt("BUTTON");
  ```

- **`attach_interrupt(std::string pinName, Func&& isr, InterruptMode mode)`** *(ESP32-only)*  
  Attaches an ISR with an argument for advanced use cases.

#### Interrupt Modes
- **`EDGE_RISING`**: Triggered when the signal rises from `LOW` to `HIGH`.  
  Use case: Detecting when a button is pressed.  
- **`EDGE_FALLING`**: Triggered when the signal falls from `HIGH` to `LOW`.  
  Use case: Detecting when a button is released.  
- **`EDGE_CHANGE`**: Triggered on any signal change (`HIGH ↔ LOW`).  
  Use case: Detecting any input state change.  
- **`LEVEL_LOW`**: Triggered while the signal remains `LOW`.  
  Use case: Monitoring a continuously low signal.  
- **`LEVEL_HIGH`**: Triggered while the signal remains `HIGH`.  
  Use case: Monitoring a continuously high signal.

---

### Platform-Specific Details
- **ESP32**
  - Supports 12-bit ADC resolution (up to 4095).
  - Allows advanced interrupt attachment with custom arguments.
- **RP2040**
  - Supports 12-bit ADC resolution.
  - Requires `PinStatus` values for interrupts.
- **Standard Arduino Boards**
  - Default 10-bit ADC resolution (up to 1023).
  - Standard interrupt modes (`RISING`, `FALLING`, etc.).

---

## Examples

### **1. LED Control**
```cpp
#include <PinSonata.hpp>

PinSonata pinManager;

void setup() {
    pinManager.configure_pins({
        {"LED", 13}
    });
    pinManager.initialize_pins();
}

void loop() {
    pinManager.toggle("LED"); // Toggle the LED state
    delay(500);               // Wait for 500ms
}
```

---

### **2. Analog Sensor Interfacing**
```cpp
#include <PinSonata.hpp>

PinSonataAnalog analogManager;

void setup() {
    Serial.begin(9600);
    analogManager.configure_pins({
        {"TEMP_SENSOR", 34} // Analog pin
    });
}

void loop() {
    float voltage = analogManager.read_voltage("TEMP_SENSOR");
    Serial.println(voltage);
    delay(1000);
}
```

---

### **3. Interrupt Handling**
```cpp
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
```

---

### **4. Data Flow**
```cpp
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
```

---

### **5. Complex Data Flow with Event Handling**
This example demonstrates a more complex workflow with discrete event-driven responses. In this scenario:
1. A button press toggles an LED and initiates a temperature reading.
2. If the temperature exceeds a threshold, a secondary LED turns on, and a warning is sent via serial.
3. The system resets if a reset button is pressed.

```cpp
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
        {"TEMP_SENSOR", 34}    // Temperature sensor
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
```

---

## Platforms Supported

- **ESP32**  
- **RP2040** (e.g., Raspberry Pi Pico)  
- **Standard Arduino Boards** (e.g., Uno, Mega, Nano)  

---

## License

This library is distributed under the MIT License. 

---