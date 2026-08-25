// ==============================================================================
// MYOSA Sensor Firmware - ESP32
// Features: High-speed ADC sampling on Core 1, I2C (BMP180/OLED) on Core 0, 
//           and UDP transmission over a Wi-Fi SoftAP.
// ==============================================================================

// --- Bisection toggles: flip to 0 to compile that peripheral OUT entirely ---
#define ENABLE_BMP180 1
#define ENABLE_OLED   1

#include <Arduino.h>
#include <Wire.h>

#if ENABLE_BMP180
#include <BarometricPressure.h>
#endif

#if ENABLE_OLED
#include <OLED.h>            
#endif

#include "soc/gpio_reg.h"   
#include "soc/io_mux_reg.h" 
#include <WiFi.h>
#include <WiFiUdp.h>

// ==============================================================================
// HARDWARE & BUFFER CONFIGURATION
// ==============================================================================

// --- Hardware Pins ---
#define PWM_PIN 4        // Pin generating the PWM signal
#define ADC_PIN 36       // Pin reading the analog sensor data

// --- Buffer & Packet Config ---
#define BUFFER_SAMPLES 512   // Number of samples per UDP packet
#define PACKET_BUFFERS 8     // Number of packets in the ring buffer

// Magic numbers used to identify valid packets on the receiving end
#define MAGIC1 0x5049434F // "PICO" in ASCII hex
#define MAGIC2 0x41444321 // "ADC!" in ASCII hex

// --- ADC Scaling ---
#define VREF_F 3.3f           // ESP32 Reference Voltage
#define ADC_COUNTS_MAX 4095.0f // 12-bit ADC max value (2^12 - 1)

// --- Wi-Fi SoftAP & UDP Config ---
const char* ssid = "MYOSA_Sensor";
const char* password = "myosa_password"; 
const int udpPort = 12345;
WiFiUDP udp;
IPAddress broadcastIp(192, 168, 4, 255); // Broadcast address for the SoftAP subnet

// ==============================================================================
// DATA STRUCTURES
// ==============================================================================

// Packet header structure. Packed to ensure no padding bytes ruin network alignment.
typedef struct __attribute__((packed)) {
    uint32_t magic1;      // First sync word
    uint32_t magic2;      // Second sync word
    uint32_t sequence;    // Packet sequence number (for tracking ordering)
    uint32_t dropped;     // Total number of dropped buffers due to overflow
    uint16_t samples;     // Number of samples in this packet payload
    uint16_t checksum;    // Simple additive checksum of the payload
    float temperature;    // Last read temperature from BMP180
    float pressure;       // Last read pressure from BMP180
} packet_header_t;

// Complete packet structure holding both header and ADC payload
typedef struct {
    packet_header_t header;
    uint16_t samples[BUFFER_SAMPLES]; // Payload: MSB holds PWM state, 15 LSBs hold ADC
    volatile bool ready;              // Flag indicating buffer is full and ready to send
} packet_t;

// ==============================================================================
// GLOBAL VARIABLES
// ==============================================================================

// Ring buffer for packets
static packet_t packets[PACKET_BUFFERS];

// State tracking variables
static volatile uint32_t sequence_number = 0;
static volatile uint32_t dropped_buffers = 0;
static volatile uint8_t write_index = 0;   // Where Core 1 is currently writing data
static volatile uint8_t read_index = 0;    // Where Core 0 is currently reading data to send
static volatile uint16_t sample_index = 0; // Current sample within the active buffer

// Sensor objects and states
BarometricPressure Pr(ULTRA_LOW_POWER);
static float current_temp = 0.0f;
static float current_pressure = 0.0f;

oLed display(128, 64);
bool display_active = false;
bool bmp_active = false;

// Variables for calculating average voltage based on PWM state
static volatile uint32_t sum_on = 0;
static volatile uint32_t sum_off = 0;
static volatile uint16_t count_on = 0;
static volatile uint16_t count_off = 0;
static volatile float led_on_avg_v = 0.0f;
static volatile float led_off_avg_v = 0.0f;

// Baseline calibration variables for breath/blow detection
#define BASELINE_CAPTURE_PACKETS 50
static volatile bool baseline_captured = false;
static volatile float baseline_delta_v = 0.0f;
static volatile uint16_t baseline_packet_count = 0;
#define BLOW_THRESHOLD_V 0.02f // Minimum voltage delta to trigger a "blow" event

// RTOS Mutex to prevent Core 0 and Core 1 from corrupting ring buffer states
portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// Calculates a simple 16-bit additive checksum for data verification
static uint16_t checksum_u16(const uint16_t *data, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

// ==============================================================================
// RTOS TASKS
// ==============================================================================

// --- Core 1 Task: High-Speed ADC Polling ---
// Continuously polls the ADC and PWM pins as fast as possible, packages the data,
// and hands it off to the UDP transmission loop.
void adcTaskFunc(void * pvParameters) {
    for(;;) {
        packet_t *packet = &packets[write_index];

        // If the current buffer hasn't been sent yet, we are dropping data
        if (packet->ready) {
            dropped_buffers++;
            vTaskDelay(1); // Yield briefly to let the UDP task catch up
            continue;
        }

        // Fast hardware read: ADC value and raw GPIO state of the PWM pin
        uint16_t adc_val = analogRead(ADC_PIN);
        uint16_t pwm_state = (REG_READ(GPIO_IN_REG) >> PWM_PIN) & 0x1;
        
        // Pack PWM state into the 16th bit (MSB) of the ADC value
        packet->samples[sample_index] = adc_val | (pwm_state << 15);
        sample_index++;

        // Accumulate data to calculate average voltages later
        if (pwm_state) {
            sum_on += adc_val;
            count_on++;
        } else {
            sum_off += adc_val;
            count_off++;
        }

        // When the buffer fills up, seal the packet and advance the write index
        if (sample_index >= BUFFER_SAMPLES) {
            // Populate packet header
            packet->header.magic1 = MAGIC1;
            packet->header.magic2 = MAGIC2;
            packet->header.sequence = sequence_number++;
            packet->header.dropped = dropped_buffers;
            packet->header.samples = BUFFER_SAMPLES;
            packet->header.checksum = checksum_u16(packet->samples, BUFFER_SAMPLES);
            
            // CRITICAL SECTION: Update ring buffer index safely
            portENTER_CRITICAL(&bufferMux);
            packet->ready = true;
            write_index++;
            if (write_index >= PACKET_BUFFERS) write_index = 0;
            portEXIT_CRITICAL(&bufferMux);
            
            sample_index = 0; // Reset for next packet

            // Calculate averages and handle baseline calibration
            if (count_on > 0 && count_off > 0) {
                led_on_avg_v = ((float)sum_on / (float)count_on) * (VREF_F / ADC_COUNTS_MAX);
                led_off_avg_v = ((float)sum_off / (float)count_off) * (VREF_F / ADC_COUNTS_MAX);

                // Initial baseline capture routine
                if (!baseline_captured) {
                    baseline_delta_v += (led_on_avg_v - led_off_avg_v);
                    baseline_packet_count++;
                    
                    if (baseline_packet_count >= BASELINE_CAPTURE_PACKETS) {
                        baseline_delta_v /= (float)baseline_packet_count; // Average the baseline
                        baseline_captured = true;
                    }
                }
            }
            
            // Reset accumulators for the next buffer
            sum_on = 0; sum_off = 0; count_on = 0; count_off = 0;
            
            // Feed the RTOS Watchdog Timer so the ESP doesn't reboot
            yield(); 
        }
    }
}

// --- Core 0 Task: Slow I2C Operations ---
// Handles the BMP180 sensor and OLED display. Pinned to Core 0 so blocking 
// I2C calls do not interrupt the high-speed ADC polling on Core 1.
void baroTaskFunc(void * pvParameters) {
    Wire.begin();
    Wire.setClock(100000); // Standard I2C speed (100kHz)
    
    // Give the I2C bus a moment to stabilize
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Retry loop for BMP180 initialization (matches MYOSA demo script logic)
    for (int i = 0; i < 5; i++) {
        bmp_active = Pr.begin();
        if (bmp_active) {
            Serial.println("BMP180 begin() OK");
            break;
        }
        Serial.println("BMP180 begin() failed. Retrying...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    if (!bmp_active) Serial.println("BMP180 permanently bypassed.");
    
    // Initialize OLED Display
    display_active = display.begin();
    if (!display_active) Serial.println("OLED begin() failed. Bypassing.");

    // Endless task loop updating at 1Hz
    for(;;) {
        // Read barometric data if sensor is connected
        if(bmp_active && Pr.ping()) {
            current_temp = Pr.getTempC();
            current_pressure = Pr.getPressurePascal();
        }

        // Update OLED UI if display is connected
        if (display_active) {
            float delta_v = led_on_avg_v - led_off_avg_v;
            bool blowing = baseline_captured && ((delta_v - baseline_delta_v) > BLOW_THRESHOLD_V);

            display.clearDisplay();
            
            // --- Row 1: Barometer Status & Temperature ---
            display.setTextSize(1);
            display.setCursor(0, 0);
            if (bmp_active) {
                display.print("Baro: OK  T: "); 
                display.print(current_temp, 1); 
                display.print("C");
            } else {
                display.print("Baro: DISCONNECTED");
            }

            // --- Row 2: Photodiode Status (> 0.2V Check) ---
            display.setCursor(0, 16);
            if (led_on_avg_v > 0.2f) {
                display.print("PD: CONN ("); 
                display.print(led_on_avg_v, 2); 
                display.print("V)");
            } else {
                display.print("PD: MISSING (<0.2V)");
            }

            // --- Row 3: Pressure / UDP Health ---
            display.setCursor(0, 32);
            if (bmp_active) {
                display.print("P: "); 
                display.print(current_pressure, 0); 
                display.print(" Pa");
            } else {
                // If baro is unplugged, show UDP buffer drop stats instead
                display.print("Drops: "); 
                display.print(dropped_buffers);
            }

            // --- Row 4: Breath Status (Large Text) ---
            display.setCursor(0, 48);
            display.setTextSize(2);
            if (!baseline_captured) display.print("CALIB...");
            else if (blowing) display.print("BLOWING");
            else display.print("IDLE");

            display.display();
        }
        
        // Delay 1 second (1Hz refresh rate)
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}

// ==============================================================================
// MAIN SETUP & LOOP
// ==============================================================================

void setup() {
    Serial.begin(115200); 

    // Setup Wi-Fi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    udp.begin(udpPort);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Initialize packet ring buffer ready states
    for (uint8_t i = 0; i < PACKET_BUFFERS; i++) packets[i].ready = false;

    // Configure PWM output and ADC resolution
    ledcAttach(PWM_PIN, 100, 12); // PWM on Pin 4, 100Hz, 12-bit resolution
    ledcWrite(PWM_PIN, 82);       // Duty cycle
    REG_SET_BIT(IO_MUX_GPIO4_REG, FUN_IE); // Ensure GPIO4 input is enabled so we can read its state
    analogReadResolution(12);     // 12-bit ADC (0-4095)

    // Launch FreeRTOS Tasks on specific cores
    // Core 0 handles slow I2C (OLED/BMP180)
    xTaskCreatePinnedToCore(baroTaskFunc, "BaroTask", 10240, NULL, 1, NULL, 0);

    // Core 1 handles fast ADC polling (replaces standard timer interrupts)
    xTaskCreatePinnedToCore(adcTaskFunc, "AdcTask", 8192, NULL, 2, NULL, 1);
}

void loop() {
    packet_t *packet = &packets[read_index];

    // If Core 1 has marked the current buffer as ready, send it over UDP
    if (packet->ready) {
        // Tag packet with the latest slow sensor readings before sending
        packet->header.temperature = current_temp;
        packet->header.pressure = current_pressure;

        // Transmit UDP Broadcast
        udp.beginPacket(broadcastIp, udpPort);
        udp.write((uint8_t*)&packet->header, sizeof(packet_header_t));
        udp.write((uint8_t*)packet->samples, BUFFER_SAMPLES * sizeof(uint16_t));
        udp.endPacket();

        // CRITICAL SECTION: Free the buffer back to Core 1 and advance index
        portENTER_CRITICAL(&bufferMux);
        packet->ready = false;
        read_index++;
        if (read_index >= PACKET_BUFFERS) read_index = 0;
        portEXIT_CRITICAL(&bufferMux);
    }
    
    // Yield to let background network processes run
    yield(); 
}