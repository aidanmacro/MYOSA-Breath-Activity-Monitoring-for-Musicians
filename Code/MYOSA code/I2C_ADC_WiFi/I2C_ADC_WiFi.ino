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

// --- Hardware Pins ---
#define PWM_PIN 4        
#define ADC_PIN 36       

// --- Buffer & Packet Config ---
#define BUFFER_SAMPLES 512
#define PACKET_BUFFERS 8

#define MAGIC1 0x5049434F 
#define MAGIC2 0x41444321 

// --- ADC scaling ---
#define VREF_F 3.3f
#define ADC_COUNTS_MAX 4095.0f

// --- Wi-Fi SoftAP & UDP Config ---
const char* ssid = "MYOSA_Sensor";
const char* password = "myosa_password"; 
const int udpPort = 12345;
WiFiUDP udp;
IPAddress broadcastIp(192, 168, 4, 255);

typedef struct __attribute__((packed)) {
    uint32_t magic1;
    uint32_t magic2;
    uint32_t sequence;
    uint32_t dropped;
    uint16_t samples;
    uint16_t checksum;
    float temperature;   
    float pressure;      
} packet_header_t;

typedef struct {
    packet_header_t header;
    uint16_t samples[BUFFER_SAMPLES];
    volatile bool ready;
} packet_t;

static packet_t packets[PACKET_BUFFERS];

static volatile uint32_t sequence_number = 0;
static volatile uint32_t dropped_buffers = 0;
static volatile uint8_t write_index = 0;
static volatile uint8_t read_index = 0;
static volatile uint16_t sample_index = 0;

BarometricPressure Pr(ULTRA_LOW_POWER);
static float current_temp = 0.0f;
static float current_pressure = 0.0f;

oLed display(128, 64);
bool display_active = false;
bool bmp_active = false;

static volatile uint32_t sum_on = 0;
static volatile uint32_t sum_off = 0;
static volatile uint16_t count_on = 0;
static volatile uint16_t count_off = 0;
static volatile float led_on_avg_v = 0.0f;
static volatile float led_off_avg_v = 0.0f;

#define BASELINE_CAPTURE_PACKETS 50
static volatile bool baseline_captured = false;
static volatile float baseline_delta_v = 0.0f;
static volatile uint16_t baseline_packet_count = 0;
#define BLOW_THRESHOLD_V 0.02f

// RTOS Mutex for safe buffer swapping
portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t checksum_u16(const uint16_t *data, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

// --- Core 1 Task: High-Speed ADC Polling ---
void adcTaskFunc(void * pvParameters) {
    for(;;) {
        packet_t *packet = &packets[write_index];

        if (packet->ready) {
            dropped_buffers++;
            vTaskDelay(1); // Yield if buffers are completely full
            continue;
        }

        // Read ADC as fast as the hardware allows
        uint16_t adc_val = analogRead(ADC_PIN);
        uint16_t pwm_state = (REG_READ(GPIO_IN_REG) >> PWM_PIN) & 0x1;
        packet->samples[sample_index] = adc_val | (pwm_state << 15);
        sample_index++;

        if (pwm_state) {
            sum_on += adc_val;
            count_on++;
        } else {
            sum_off += adc_val;
            count_off++;
        }

        if (sample_index >= BUFFER_SAMPLES) {
            packet->header.magic1 = MAGIC1;
            packet->header.magic2 = MAGIC2;
            packet->header.sequence = sequence_number++;
            packet->header.dropped = dropped_buffers;
            packet->header.samples = BUFFER_SAMPLES;
            packet->header.checksum = checksum_u16(packet->samples, BUFFER_SAMPLES);
            
            portENTER_CRITICAL(&bufferMux);
            packet->ready = true;
            write_index++;
            if (write_index >= PACKET_BUFFERS) write_index = 0;
            portEXIT_CRITICAL(&bufferMux);
            
            sample_index = 0;

            if (count_on > 0 && count_off > 0) {
                led_on_avg_v = ((float)sum_on / (float)count_on) * (VREF_F / ADC_COUNTS_MAX);
                led_off_avg_v = ((float)sum_off / (float)count_off) * (VREF_F / ADC_COUNTS_MAX);

                if (!baseline_captured) {
                    baseline_delta_v += (led_on_avg_v - led_off_avg_v);
                    baseline_packet_count++;
                    if (baseline_packet_count >= BASELINE_CAPTURE_PACKETS) {
                        baseline_delta_v /= (float)baseline_packet_count;
                        baseline_captured = true;
                    }
                }
            }
            sum_on = 0; sum_off = 0; count_on = 0; count_off = 0;
            
            // Feed the Task Watchdog Timer
            yield(); 
        }
    }
}

// --- Core 0 Task for I2C ---
void baroTaskFunc(void * pvParameters) {
    Wire.begin();
    Wire.setClock(100000);
    
    // Give the I2C bus a moment to stabilize
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Retry loop matching the MYOSA demo script
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
    
    display_active = display.begin();
    if (!display_active) Serial.println("OLED begin() failed. Bypassing.");

    for(;;) {
        if(bmp_active && Pr.ping()) {
            current_temp = Pr.getTempC();
            current_pressure = Pr.getPressurePascal();
        }

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
                // If baro is unplugged, use this line to show UDP drops instead
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
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Run at a safe 1Hz rate
    }
}

void setup() {
    Serial.begin(115200); 

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    udp.begin(udpPort);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    for (uint8_t i = 0; i < PACKET_BUFFERS; i++) packets[i].ready = false;

    ledcAttach(PWM_PIN, 100, 12); 
    ledcWrite(PWM_PIN, 82); 
    REG_SET_BIT(IO_MUX_GPIO4_REG, FUN_IE);
    analogReadResolution(12); 

    // I2C Task on Core 0
    xTaskCreatePinnedToCore(baroTaskFunc, "BaroTask", 10240, NULL, 1, NULL, 0);

    // ADC Polling Task on Core 1 (Replacing Timer Interrupt)
    xTaskCreatePinnedToCore(adcTaskFunc, "AdcTask", 8192, NULL, 2, NULL, 1);
}

void loop() {
    packet_t *packet = &packets[read_index];

    if (packet->ready) {
        packet->header.temperature = current_temp;
        packet->header.pressure = current_pressure;

        udp.beginPacket(broadcastIp, udpPort);
        udp.write((uint8_t*)&packet->header, sizeof(packet_header_t));
        udp.write((uint8_t*)packet->samples, BUFFER_SAMPLES * sizeof(uint16_t));
        udp.endPacket();

        portENTER_CRITICAL(&bufferMux);
        packet->ready = false;
        read_index++;
        if (read_index >= PACKET_BUFFERS) read_index = 0;
        portEXIT_CRITICAL(&bufferMux);
    }
    yield(); 
}