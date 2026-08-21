#include <Arduino.h>
#include <Wire.h>
#include <BarometricPressure.h>
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
#define SAMPLE_PERIOD_US 4 

#define MAGIC1 0x5049434F 
#define MAGIC2 0x41444321 

// --- Wi-Fi SoftAP & UDP Config ---
const char* ssid = "MYOSA_Sensor";
const char* password = "myosa_password"; // Must be at least 8 characters
const int udpPort = 12345;
WiFiUDP udp;

// Broadcast to all devices connected to the ESP32's SoftAP (default subnet 192.168.4.x)
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

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t checksum_u16(const uint16_t *data, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    
    packet_t *packet = &packets[write_index];

    if (packet->ready) {
        dropped_buffers++;
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    uint16_t adc_val = analogRead(ADC_PIN);
    uint16_t pwm_state = (REG_READ(GPIO_IN_REG) >> PWM_PIN) & 0x1;
    packet->samples[sample_index] = adc_val | (pwm_state << 15);
    sample_index++;

    if (sample_index >= BUFFER_SAMPLES) {
        packet->header.magic1 = MAGIC1;
        packet->header.magic2 = MAGIC2;
        packet->header.sequence = sequence_number++;
        packet->header.dropped = dropped_buffers;
        packet->header.samples = BUFFER_SAMPLES;
        packet->header.checksum = checksum_u16(packet->samples, BUFFER_SAMPLES);
        
        packet->ready = true;
        sample_index = 0;

        write_index++;
        if (write_index >= PACKET_BUFFERS) {
            write_index = 0;
        }
    }
    
    portEXIT_CRITICAL_ISR(&timerMux);
}

// --- Core 0 Task for I2C ---
void baroTaskFunc(void * pvParameters) {
    Wire.begin();
    Wire.setClock(100000);
    Pr.begin();

    for(;;) {
        if(Pr.ping()) {
            current_temp = Pr.getTempC(false);
            current_pressure = Pr.getPressurePascal(false);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200); 

    // --- Wi-Fi SoftAP Setup ---
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    udp.begin(udpPort);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    for (uint8_t i = 0; i < PACKET_BUFFERS; i++) {
        packets[i].ready = false;
    }

    // PWM Setup
    ledcAttach(PWM_PIN, 100, 12); 
    ledcWrite(PWM_PIN, 82);  // MUST BE 100Hz 2%

    REG_SET_BIT(IO_MUX_GPIO4_REG, FUN_IE);

    analogReadResolution(12); 

    xTaskCreatePinnedToCore(
        baroTaskFunc, "BaroTask", 4096, NULL, 1, NULL, 0 
    );

    // Timer Setup 
    timer = timerBegin(1000000); 
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, SAMPLE_PERIOD_US, true, 0); 
}

void loop() {
    packet_t *packet = &packets[read_index];

    if (packet->ready) {
        packet->header.temperature = current_temp;
        packet->header.pressure = current_pressure;

        // Send via UDP instead of Serial
        udp.beginPacket(broadcastIp, udpPort);
        udp.write((uint8_t*)&packet->header, sizeof(packet_header_t));
        udp.write((uint8_t*)packet->samples, BUFFER_SAMPLES * sizeof(uint16_t));
        udp.endPacket();

        portENTER_CRITICAL(&timerMux);
        packet->ready = false;
        portEXIT_CRITICAL(&timerMux);

        read_index++;
        if (read_index >= PACKET_BUFFERS) {
            read_index = 0;
        }
    }
    
    yield(); 
}