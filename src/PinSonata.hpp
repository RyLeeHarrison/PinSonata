//
//    FILE: PinSonata.hpp
//  AUTHOR: RyLee Harrison
// VERSION: 1.0.0
//     URL: https://github.com/RyLeeHarrison/PinSonata

#ifndef PINSONATA_HPP
#define PINSONATA_HPP

#include <map>
#include <string>
#include <functional>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
    #define PINSONATA_HAS_YIELD
    #define PINSONATA_ADC_BITS 12
    #define PINSONATA_ADC_MAX 4095
    #define PINSONATA_HAS_CUSTOM_ADC_RESOLUTION
    #define PINSONATA_HAS_INTERRUPT_ARG
    #define PINSONATA_USE_GPIO_INTERRUPTS
    #define PINSONATA_REFERENCE_VOLTAGE 3.3f
    
    // ESP32 specific interrupt modes
    #define PINSONATA_INT_RISING    0x01
    #define PINSONATA_INT_FALLING   0x02
    #define PINSONATA_INT_CHANGE    0x03
    #define PINSONATA_INT_ONLOW     0x04
    #define PINSONATA_INT_ONHIGH    0x05

#elif defined(ARDUINO_ARCH_RP2040)
    #define PINSONATA_ADC_BITS 12
    #define PINSONATA_ADC_MAX 4095
    #define PINSONATA_REFERENCE_VOLTAGE 3.3f
    #define PINSONATA_USE_GPIO_INTERRUPTS
    
    // RP2040 uses PinStatus enum values
    #define PINSONATA_INT_RISING    RISING    // 4
    #define PINSONATA_INT_FALLING   FALLING   // 3
    #define PINSONATA_INT_CHANGE    CHANGE    // 2
    #define PINSONATA_INT_ONLOW     LOW       // 0
    #define PINSONATA_INT_ONHIGH    HIGH      // 1

#else
    // Default configuration for unknown platforms
    #define PINSONATA_ADC_BITS 10
    #define PINSONATA_ADC_MAX 1023
    #define PINSONATA_REFERENCE_VOLTAGE 5.0f
    #define PINSONATA_USE_PIN_INTERRUPTS
    
    // Standard Arduino interrupt modes
    #define PINSONATA_INT_RISING    RISING
    #define PINSONATA_INT_FALLING   FALLING
    #define PINSONATA_INT_CHANGE    CHANGE
    #define PINSONATA_INT_ONLOW     LOW
    #define PINSONATA_INT_ONHIGH    HIGH
#endif

// Helper macros
#ifdef PINSONATA_HAS_YIELD
    #define PINSONATA_YIELD() yield()
#else
    #define PINSONATA_YIELD() (void)0
#endif

// Platform-independent interrupt attachment
#ifdef PINSONATA_USE_GPIO_INTERRUPTS
    #if defined(ARDUINO_ARCH_RP2040)
        #define PINSONATA_ATTACH_INTERRUPT(gpio, isr, mode) \
            attachInterrupt(gpio, isr, static_cast<PinStatus>(mode))
    #else
        #define PINSONATA_ATTACH_INTERRUPT(gpio, isr, mode) \
            attachInterrupt(gpio, isr, static_cast<int>(mode))
    #endif
    #define PINSONATA_DETACH_INTERRUPT(gpio) detachInterrupt(gpio)
#else
    #if defined(ARDUINO_ARCH_RP2040)
        #define PINSONATA_ATTACH_INTERRUPT(gpio, isr, mode) \
            attachInterrupt(digitalPinToInterrupt(gpio), isr, static_cast<PinStatus>(mode))
    #else
        #define PINSONATA_ATTACH_INTERRUPT(gpio, isr, mode) \
            attachInterrupt(digitalPinToInterrupt(gpio), isr, static_cast<int>(mode))
    #endif
    #define PINSONATA_DETACH_INTERRUPT(gpio) \
        detachInterrupt(digitalPinToInterrupt(gpio))
#endif


class PinSonata {
public:
    PinSonata() = default;

    void configure_pins(const std::map<std::string, int>& newPinMap) {
        pinMap = newPinMap;
    }

    void initialize_pins() {
        // Changed from structured binding to traditional iterator
        for (auto it = pinMap.begin(); it != pinMap.end(); ++it) {
            pinMode(it->second, OUTPUT);
            digitalWrite(it->second, LOW);
            pinStates[it->first] = LOW;
        }
    }

    bool wait_for_state(const std::string& pin, int desiredState, unsigned long timeout_ms = 1000) {
        if (!has_pin(pin)) return false;

        unsigned long startTime = millis();
        while (read(pin) != desiredState) {
            if (timeout_ms > 0 && (millis() - startTime) >= timeout_ms) {
                return false;
            }
            PINSONATA_YIELD();
        }
        return true;
    }

    bool wait_for_change(const std::string& pin, unsigned long timeout_ms = 1000) {
        if (!has_pin(pin)) return false;

        int initialState = read(pin);
        unsigned long startTime = millis();

        while (read(pin) == initialState) {
            if (timeout_ms > 0 && (millis() - startTime) >= timeout_ms) {
                return false;
            }
            PINSONATA_YIELD();
        }
        return true;
    }

    void set_pin_mode(const std::string& pin, uint8_t mode) {
        if (has_pin(pin)) {
            pinMode(pinMap[pin], mode);
        }
    }

    void operator()(const std::string& pin, int state) {
        if (has_pin(pin)) {
            digitalWrite(pinMap[pin], state);
            pinStates[pin] = state;
        }
    }

    int read(const std::string& pin) {
        if (has_pin(pin)) {
            return digitalRead(pinMap[pin]);
        }
        return -1;
    }

    int get_state(const std::string& pin) {
        return has_pin(pin) ? pinStates[pin] : -1;
    }

    void toggle(const std::string& pin) {
        if (has_pin(pin)) {
            int newState = pinStates[pin] == HIGH ? LOW : HIGH;
            (*this)(pin, newState);
        }
    }

    void pulse(const std::string& pin, unsigned long duration) {
        if (has_pin(pin)) {
            unsigned long startTime = micros();
            digitalWrite(pinMap[pin], HIGH);
            while ((micros() - startTime) < duration) {
                PINSONATA_YIELD();
            }
            digitalWrite(pinMap[pin], LOW);
            pinStates[pin] = LOW;
        }
    }

    bool has_pin(const std::string& pin) const {
        return pinMap.find(pin) != pinMap.end();
    }

    int get_gpio(const std::string& pin) const {
        auto it = pinMap.find(pin);
        return it != pinMap.end() ? it->second : -1;
    }

protected:
    std::map<std::string, int> pinStates;
    std::map<std::string, int> pinMap;
};

// Analog functionality
class PinSonataAnalog : public PinSonata {
public:
    enum class AdcResolution {
        BITS_9 = 9,
        BITS_10 = 10,
        BITS_11 = 11,
        BITS_12 = 12
    };

    PinSonataAnalog() = default;

    void set_adc_resolution(AdcResolution bits) {
#ifdef PINSONATA_HAS_CUSTOM_ADC_RESOLUTION
        analogReadResolution(static_cast<int>(bits));
#endif
    }

    int read_analog(const std::string& pin) {
        if (!has_pin(pin)) return -1;
        return analogRead(get_gpio(pin));
    }

    float read_voltage(const std::string& pin, float reference_voltage = PINSONATA_REFERENCE_VOLTAGE) {
        if (!has_pin(pin)) return -1.0f;
        int raw = analogRead(get_gpio(pin));
        return (raw * reference_voltage) / PINSONATA_ADC_MAX;
    }

    int read_analog_averaged(const std::string& pin, uint8_t samples = 10, uint8_t delay_ms = 1) {
        if (!has_pin(pin)) return -1;

        int gpio = get_gpio(pin);
        long sum = 0;
        for (uint8_t i = 0; i < samples; i++) {
            sum += analogRead(gpio);
            if (delay_ms > 0) delay(delay_ms);
            PINSONATA_YIELD();
        }
        return sum / samples;
    }
};

// Interrupt functionality
class PinSonataInterrupt : public PinSonata {
public:
     enum class InterruptMode {
        LEVEL_LOW = PINSONATA_INT_ONLOW,
        LEVEL_HIGH = PINSONATA_INT_ONHIGH,
        EDGE_FALLING = PINSONATA_INT_FALLING,
        EDGE_RISING = PINSONATA_INT_RISING,
        EDGE_CHANGE = PINSONATA_INT_CHANGE
    };

    PinSonataInterrupt() = default;

    bool attach_interrupt(const std::string& pin, void (*isr)(), InterruptMode mode) {
        if (!has_pin(pin)) return false;
        int gpio = get_gpio(pin);
        
        #if defined(ARDUINO_ARCH_RP2040)
            attachInterrupt(gpio, isr, static_cast<PinStatus>(mode));
        #elif defined(ARDUINO_ARCH_ESP32)
            attachInterrupt(gpio, isr, static_cast<int>(mode));
        #else
            int interruptNum = digitalPinToInterrupt(gpio);
            if (interruptNum == NOT_AN_INTERRUPT) return false;
            attachInterrupt(interruptNum, isr, static_cast<int>(mode));
        #endif
        return true;
    }


#ifdef PINSONATA_HAS_INTERRUPT_ARG
    template<typename Func>
    bool attach_interrupt(const std::string& pin, Func&& isr, InterruptMode mode) {
        if (!has_pin(pin)) return false;
        int gpio = get_gpio(pin);

        interruptHandlers[gpio] = std::forward<Func>(isr);

        auto wrapper = [](void* arg) {
            int gpio = *static_cast<int*>(arg);
            if (interruptHandlers.count(gpio)) {
                interruptHandlers[gpio]();
            }
        };

        int* arg = new int(gpio);
        interruptArgs[gpio] = arg;

        #if defined(ARDUINO_ARCH_ESP32)
            attachInterruptArg(gpio, wrapper, arg, static_cast<int>(mode));
        #endif
        return true;
    }
#endif

    bool detach_interrupt(const std::string& pin) {
        if (!has_pin(pin)) return false;
        int gpio = get_gpio(pin);
        
        PINSONATA_DETACH_INTERRUPT(gpio);
        
        interruptHandlers.erase(gpio);
#ifdef PINSONATA_HAS_INTERRUPT_ARG
        if (interruptArgs.count(gpio)) {
            delete interruptArgs[gpio];
            interruptArgs.erase(gpio);
        }
#endif
        return true;
    }

private:
    static std::map<int, std::function<void()>> interruptHandlers;
#ifdef PINSONATA_HAS_INTERRUPT_ARG
    static std::map<int, int*> interruptArgs;
#endif
};

// Initialize static members
std::map<int, std::function<void()>> PinSonataInterrupt::interruptHandlers;
#ifdef PINSONATA_HAS_INTERRUPT_ARG
std::map<int, int*> PinSonataInterrupt::interruptArgs;
#endif

#endif // PINSONATA_HPP