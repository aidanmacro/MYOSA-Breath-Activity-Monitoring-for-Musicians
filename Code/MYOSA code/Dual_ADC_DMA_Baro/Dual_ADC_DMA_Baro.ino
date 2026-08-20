#include <Arduino.h>
#include <Wire.h>
#include <BarometricPressure.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_adc/adc_continuous.h" // Replaces driver/i2s.h in Core v3

// --- Hardware Pins ---
#define PWM_PIN 4
#define ADC_PD_PIN 36  // ADC1 Channel 0
#define ADC_PWM_PIN 39 // ADC1 Channel 3

// --- Buffer & Packet Config ---
#define BUFFER_SAMPLES 700 
#define PACKET_BUFFERS 8
#define ADC_SAMPLE_RATE 100000 // 100kHz per channel 

#define MAGIC1 0x5049434F 
#define MAGIC2 0x41444321 

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

static uint32_t sequence_number = 0;
BarometricPressure Pr(ULTRA_LOW_POWER);
static float current_temp = 0.0f;
static float current_pressure = 0.0f;

adc_continuous_handle_t adc_handle = NULL;

static uint16_t checksum_u16(const uint16_t *data, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

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

void configure_adc_dma() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUFFER_SAMPLES * sizeof(uint16_t) * PACKET_BUFFERS,
        .conv_frame_size = BUFFER_SAMPLES * sizeof(uint16_t), 
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 2,
        .adc_pattern = NULL, // Assigned directly below
        .sample_freq_hz = ADC_SAMPLE_RATE * 2, // 200kHz total (100kHz per channel)
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    
    adc_digi_pattern_config_t adc_pattern[2];
    
    // Pattern 0: Channel 0 (GPIO 36)
    adc_pattern[0].atten = ADC_ATTEN_DB_12; 
    adc_pattern[0].channel = ADC_CHANNEL_0;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    // Pattern 1: Channel 3 (GPIO 39)
    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = ADC_CHANNEL_3;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    dig_cfg.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void setup() {
    Serial.begin(115200); 

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    udp.begin(udpPort);

    ledcAttach(PWM_PIN, 100, 12); 
    ledcWrite(PWM_PIN, 82); 

    xTaskCreatePinnedToCore(baroTaskFunc, "BaroTask", 4096, NULL, 1, NULL, 0);
    configure_adc_dma();
}

void loop() {
    packet_header_t header;
    uint16_t dma_buffer[BUFFER_SAMPLES]; 
    uint32_t bytes_read = 0;

    // Block until DMA completely fills the 1400-byte frame
    esp_err_t ret = adc_continuous_read(adc_handle, (uint8_t*)dma_buffer, sizeof(dma_buffer), &bytes_read, portMAX_DELAY);

    if (ret == ESP_OK && bytes_read == sizeof(dma_buffer)) {
        header.magic1 = MAGIC1;
        header.magic2 = MAGIC2;
        header.sequence = sequence_number++;
        header.dropped = 0; 
        header.samples = BUFFER_SAMPLES;
        header.checksum = checksum_u16(dma_buffer, BUFFER_SAMPLES);
        header.temperature = current_temp;
        header.pressure = current_pressure;

        udp.beginPacket(broadcastIp, udpPort);
        udp.write((uint8_t*)&header, sizeof(packet_header_t));
        udp.write((uint8_t*)dma_buffer, sizeof(dma_buffer));
        udp.endPacket();
    }
}