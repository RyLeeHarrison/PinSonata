#include <Arduino.h>
#include <PinSonata.hpp>

// Mock Arduino functions for testing if needed
#ifndef ARDUINO
unsigned long _millis = 0;
unsigned long millis() { return _millis; }
void delay(unsigned long ms) { _millis += ms; }
#endif

class PinSonataTest {
public:
  static void run_all_tests() {
    Serial.println("\nStarting PinSonata Tests...");
    
    test_basic_digital_operations();
    test_analog_operations();
    test_interrupt_operations();
    test_timing_operations();
    test_error_conditions();
    
    Serial.println("\nAll tests completed!");
    Serial.print("Passed: "); Serial.println(passed_tests);
    Serial.print("Failed: "); Serial.println(failed_tests);
  }

private:
  static void test_basic_digital_operations() {
    Serial.println("\n=== Testing Basic Digital Operations ===");
    
    PinSonata ps;
    std::map<std::string, int> pinMap = {
      {"led", LED_BUILTIN},  // Use built-in LED for more reliable testing
      {"button", 2}
    };
    
    // Test pin configuration
    ps.configure_pins(pinMap);
    assert_true("Pin map configuration", ps.has_pin("led"));
    assert_true("GPIO retrieval", ps.get_gpio("led") == LED_BUILTIN);
    assert_true("Non-existent pin check", !ps.has_pin("fake"));
    
    // Initialize pins to known state
    ps.initialize_pins();
    delay(1);  // Allow pins to settle
    
    // Test digital write operations
    ps("led", HIGH);
    delay(1);  // Allow pin to settle
    assert_true("Digital write HIGH sets state", ps.get_state("led") == HIGH);
    assert_true("Digital write HIGH reflects on pin", ps.read("led") == HIGH);
    
    ps("led", LOW);
    delay(1);  // Allow pin to settle
    assert_true("Digital write LOW sets state", ps.get_state("led") == LOW);
    assert_true("Digital write LOW reflects on pin", ps.read("led") == LOW);
    
    // Test toggle operation
    ps.toggle("led");
    delay(1);  // Allow pin to settle
    assert_true("Toggle HIGH sets state", ps.get_state("led") == HIGH);
    assert_true("Toggle HIGH reflects on pin", ps.read("led") == HIGH);
    
    ps.toggle("led");
    delay(1);  // Allow pin to settle
    assert_true("Toggle LOW sets state", ps.get_state("led") == LOW);
    assert_true("Toggle LOW reflects on pin", ps.read("led") == LOW);
  }

  static void test_analog_operations() {
    Serial.println("\n=== Testing Analog Operations ===");
    
    PinSonataAnalog psa;
    std::map<std::string, int> pinMap = {
      {"sensor", 34}  // Assuming GPIO34 is an ADC pin
    };
    
    psa.configure_pins(pinMap);
    
    // Test ADC resolution setting
    psa.set_adc_resolution(PinSonataAnalog::AdcResolution::BITS_12);
    
    // Test analog read (note: actual values will depend on hardware)
    int analog_value = psa.read_analog("sensor");
    assert_true("Analog read range", analog_value >= 0 && analog_value <= 4095);
    
    // Test averaged reading
    int averaged_value = psa.read_analog_averaged("sensor", 5, 1);
    assert_true("Averaged analog read range", averaged_value >= 0 && averaged_value <= 4095);
    
    // Test voltage reading (assuming 3.3V reference)
    float voltage = psa.read_voltage("sensor");
    assert_true("Voltage read range", voltage >= 0.0f && voltage <= 3.3f);
  }

  static void test_interrupt_operations() {
    Serial.println("\n=== Testing Interrupt Operations ===");
    
    PinSonataInterrupt psi;
    std::map<std::string, int> pinMap = {
      {"button", 2}  // Assuming GPIO2 supports interrupts
    };
    
    psi.configure_pins(pinMap);
    
    // Test interrupt attachment
    bool interrupt_attached = psi.attach_interrupt("button", 
      []() { /* Empty handler */ }, 
      PinSonataInterrupt::InterruptMode::EDGE_RISING
    );
    assert_true("Interrupt attachment", interrupt_attached);
    
    // Test invalid pin interrupt attachment
    bool invalid_interrupt = psi.attach_interrupt("invalid_pin", 
      []() { /* Empty handler */ }, 
      PinSonataInterrupt::InterruptMode::EDGE_RISING
    );
    assert_true("Invalid pin interrupt rejection", !invalid_interrupt);
    
    // Test interrupt detachment
    bool interrupt_detached = psi.detach_interrupt("button");
    assert_true("Interrupt detachment", interrupt_detached);
  }

  static void test_timing_operations() {
    Serial.println("\n=== Testing Timing Operations ===");
    
    PinSonata ps;
    std::map<std::string, int> pinMap = {
      {"test_pin", LED_BUILTIN}  // Use built-in LED for more reliable testing
    };
    
    ps.configure_pins(pinMap);
    ps.initialize_pins();
    
    // Test wait_for_state with timeout
    unsigned long start = millis();
    bool timeout_occurred = !ps.wait_for_state("test_pin", HIGH, 100);
    unsigned long duration = millis() - start;
    
    assert_true("Wait timeout occurred", timeout_occurred);
    assert_true("Wait timeout duration", duration >= 100);
    
    // Test pulse timing
    start = millis();
    ps.pulse("test_pin", 1000);  // 1ms pulse
    duration = millis() - start;
    
    assert_true("Pulse duration", duration >= 1);
    delay(1);  // Allow pin to settle
    assert_true("Pulse end state", ps.get_state("test_pin") == LOW);
  }

  static void test_error_conditions() {
    Serial.println("\n=== Testing Error Conditions ===");
    
    PinSonata ps;
    
    // Test operations on unconfigured pins
    assert_true("Read unconfigured pin", ps.read("undefined") == -1);
    assert_true("Get state unconfigured pin", ps.get_state("undefined") == -1);
    assert_true("Get GPIO unconfigured pin", ps.get_gpio("undefined") == -1);
    
    // Test empty pin map
    std::map<std::string, int> emptyMap;
    ps.configure_pins(emptyMap);
    assert_true("Empty pin map handling", !ps.has_pin("any"));
  }

  static void assert_true(const char* test_name, bool condition) {
    if (condition) {
      Serial.print("✓ PASS: ");
      passed_tests++;
    } else {
      Serial.print("✗ FAIL: ");
      failed_tests++;
    }
    Serial.println(test_name);
  }

  static int passed_tests;
  static int failed_tests;
};

// Initialize static members
int PinSonataTest::passed_tests = 0;
int PinSonataTest::failed_tests = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  PinSonataTest::run_all_tests();
}

void loop() {
  // Empty loop
}