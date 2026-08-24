#include <Arduino.h>



// --- Hardware Pins ---

#define PWM_PIN 4         // Pin driving the LED

#define ADC_PIN 36        // ADC1 pin, read via the continuous/DMA ADC driver



// --- Buffer & Packet Config ---

#define BUFFER_SAMPLES 512

#define PACKET_BUFFERS 8



// Requested ADC sampling frequency in Hz. This now drives the DMA-backed

// continuous ADC peripheral directly (no more hw_timer + analogRead()).

// The ESP32's continuous ADC (I2S0 used as DMA FIFO) reliably sustains

// single-channel rates up to roughly 200 kHz in practice; pushing much

// higher risks the DMA pool overflowing faster than samplerTask can

// drain it (which shows up as rising `dropped` counts). Tune to taste.

#define SAMPLE_RATE_HZ 200000



// Magic values expected by the Python host script's wait_for_magic() (b"OCIP!CDA")

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

// sequence_number/dropped_buffers are only ever touched by samplerTask now.

// write_index is only written by samplerTask, read_index only by txTask -

// each task owns its own index, and `ready` is a single-writer/single-reader

// flag per slot (samplerTask sets true, txTask clears it), so this stays

// lock-free across the two cores without needing portMUX critical sections.

static uint32_t sequence_number = 0;

static uint32_t dropped_buffers = 0;

static volatile uint8_t write_index = 0;

static volatile uint8_t read_index = 0;



// --- ADC continuous (DMA) config ---

static uint8_t adc_pins[] = { ADC_PIN };

static const uint8_t adc_pins_count = 1;



static TaskHandle_t samplerTaskHandle = NULL;

static TaskHandle_t txTaskHandle = NULL;



// --- Helper Functions ---

static uint16_t checksum_u16(const uint16_t *data, uint16_t n) {

    uint32_t sum = 0;

    for (uint16_t i = 0; i < n; i++) {

        sum += data[i];

    }

    return (uint16_t)(sum & 0xFFFF);

}



// Fires in ISR context once per completed DMA conversion frame

// (BUFFER_SAMPLES conversions, since conversions_per_pin == BUFFER_SAMPLES

// below). Kept minimal on purpose - just wake the sampler task.

void ARDUINO_ISR_ATTR adcComplete() {

    BaseType_t mustYield = pdFALSE;

    vTaskNotifyGiveFromISR(samplerTaskHandle, &mustYield);

    portYIELD_FROM_ISR(mustYield);

}



// --- Core 1 task: drains finished DMA frames from the ADC driver and

// packs them into the ring buffer. This replaces the old onTimer() ISR. ---

void samplerTask(void *pvParameters) {

    adc_continuous_result_t *result = NULL;



    for (;;) {

        // Sleep until the ISR signals a frame is ready

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);



        // Drain everything currently queued in the driver's DMA pool

        // before going back to sleep, so we never fall behind it.

        while (analogContinuousRead(&result, 0)) {

            packet_t *packet = &packets[write_index];



            if (packet->ready) {

                // txTask hasn't caught up on this slot yet - drop the frame.

                dropped_buffers++;

            } else {

                for (uint16_t i = 0; i < BUFFER_SAMPLES; i++) {

                    packet->samples[i] = (uint16_t)result[i].avg_read_raw;

                }



                packet->header.magic1 = MAGIC1;

                packet->header.magic2 = MAGIC2;

                packet->header.sequence = sequence_number++;

                packet->header.dropped = dropped_buffers;

                packet->header.samples = BUFFER_SAMPLES;

                packet->header.checksum = checksum_u16(packet->samples, BUFFER_SAMPLES);



                packet->ready = true;



                write_index = (write_index + 1) % PACKET_BUFFERS;

            }

        }

    }

}



// --- Core 0 task: sends finished packets over Serial. Living on its own

// core means a slow/blocking UART write can never stall ADC sampling. ---

void txTask(void *pvParameters) {

    for (;;) {

        packet_t *packet = &packets[read_index];



        if (packet->ready) {

            Serial.write((uint8_t *)&packet->header, sizeof(packet_header_t));

            Serial.write((uint8_t *)packet->samples, BUFFER_SAMPLES * sizeof(uint16_t));



            packet->ready = false;

            read_index = (read_index + 1) % PACKET_BUFFERS;

        } else {

            vTaskDelay(1); // nothing queued - don't busy-spin

        }

    }

}



// --- Setup ---

void setup() {

    // Matched to the BAUD = 115200 setting in the Python script

    Serial.begin(2000000);



    for (uint8_t i = 0; i < PACKET_BUFFERS; i++) {

        packets[i].ready = false;

    }



    // --- LED PWM setup (unrelated to the ADC/DMA path) ---

    ledcAttach(PWM_PIN, 100, 12);

    ledcWrite(PWM_PIN, 81); // ~1% duty at 12-bit resolution



    // --- ADC continuous (DMA) setup ---

    analogContinuousSetWidth(12);        // 0-4095, matches the script's expected 12-bit max

    analogContinuousSetAtten(ADC_11db);  // full ~0-3.3V input range



    // pins[], pin_count, conversions_per_pin (one full BUFFER_SAMPLES frame

    // per callback), sampling frequency, ISR callback

    analogContinuous(adc_pins, adc_pins_count, BUFFER_SAMPLES, SAMPLE_RATE_HZ, &adcComplete);



    // Create the worker tasks BEFORE starting the ADC so samplerTaskHandle

    // exists before the ISR can possibly fire. Sampler pinned to core 1

    // (freed up below since loop() deletes itself), transmitter to core 0.

    xTaskCreatePinnedToCore(samplerTask, "adc_sampler", 4096, NULL, 3, &samplerTaskHandle, 1);

    xTaskCreatePinnedToCore(txTask, "uart_tx", 4096, NULL, 2, &txTaskHandle, 0);



    analogContinuousStart();

}



// --- Main Loop ---

// All real work now lives in samplerTask (core 1) and txTask (core 0), so

// free up the Arduino loop task's core entirely rather than idling it.

void loop() {

    vTaskDelete(NULL);

}


