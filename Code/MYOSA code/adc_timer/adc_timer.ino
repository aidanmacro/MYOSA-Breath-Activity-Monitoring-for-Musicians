#include <Arduino.h>

// --- Hardware Pins ---
#define PWM_PIN 4        // Pin driving the LED
#define ADC_PIN 36       // ADC pin for photodiode/TIA (Use ADC1 pins like 32, 33, 34, 35)

// --- Buffer & Packet Config ---
#define BUFFER_SAMPLES 512
#define PACKET_BUFFERS 8

// Timer interval in microseconds (2us = 500kHz).
#define SAMPLE_PERIOD_US 4

// These match the b"OCIP!CDA" expected by the Python script's wait_for_magic()
#define MAGIC1 0x5049434F 
#define MAGIC2 0x41444321 

// --- Packet Structures ---
typedef struct __attribute__((packed)) {
    uint32_t magic1;
    uint32_t magic2;
    uint32_t sequence;
    uint32_t dropped;
    uint16_t samples;
    uint16_t checksum;
} packet_header_t;

typedef struct {
    packet_header_t header;
    uint16_t samples[BUFFER_SAMPLES];
    volatile bool ready;
} packet_t;

static packet_t packets[PACKET_BUFFERS];

// --- State Variables ---
static volatile uint32_t sequence_number = 0;
static volatile uint32_t dropped_buffers = 0;
static volatile uint8_t write_index = 0;
static volatile uint8_t read_index = 0;
static volatile uint16_t sample_index = 0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Helper Functions ---
static uint16_t checksum_u16(const uint16_t *data, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

// --- ADC Timer Interrupt Service Routine ---
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    
    packet_t *packet = &packets[write_index];

    // If the current packet is already marked ready (hasn't been sent yet), we drop data.
    if (packet->ready) {
        dropped_buffers++;
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    // Read ADC and store (The Python script expects max 4095, so we use 12-bit)
    packet->samples[sample_index] = analogRead(ADC_PIN);
    sample_index++;

    // Packet full? Finalize and move to next buffer
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

// --- Setup ---
void setup() {
    // Matched to the BAUD = 115200 setting in the Python script
    Serial.begin(115200); 

    for (uint8_t i = 0; i < PACKET_BUFFERS; i++) {
        packets[i].ready = false;
    }

    // --- Core 3.x PWM Setup ---
    // ledcAttach(pin, frequency, resolution)
    ledcAttach(PWM_PIN, 100, 12); 
    
    // ledcWrite(pin, duty_cycle)
    // 12-bit resolution means 0-4095. 1% of 4096 is ~41.
    ledcWrite(PWM_PIN, 532); 

    // Setup ADC
    analogReadResolution(12); // Ensures max value is 4095, as expected by the script

    // --- Core 3.x Timer Setup ---
    // timerBegin(frequency) -> 1,000,000 Hz = 1 tick per microsecond
    timer = timerBegin(1000000); 
    
    // timerAttachInterrupt(timer_instance, ISR)
    timerAttachInterrupt(timer, &onTimer);
    
    // timerAlarm(timer_instance, alarm_value, autoreload, reload_count)
    // Fire every SAMPLE_PERIOD_US, true for auto-reload, 0 for infinite reloads
    timerAlarm(timer, SAMPLE_PERIOD_US, true, 0); 
}

// --- Main Loop ---
void loop() {
    packet_t *packet = &packets[read_index];

    if (packet->ready) {
        // Send Header
        Serial.write((uint8_t*)&packet->header, sizeof(packet_header_t));
        
        // Send Samples
        Serial.write((uint8_t*)packet->samples, BUFFER_SAMPLES * sizeof(uint16_t));

        // Mark as processed
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