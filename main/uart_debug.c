#include "uart_debug.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define TELEMETRY_QUEUE_LEN 8
#define UART_DEBUG_TASK_STACK 3072
#define UART_DEBUG_TASK_PRIORITY 5

typedef struct __attribute__((packed)) {
    uint8_t sync[2];
    uint32_t seq;
    uint32_t timestamp_us;
    float q[4];
    float angular_rate[3];
    float gyro_bias[3];
    float accel_body[3];
    uint16_t crc;
} telemetry_packet_t;

static QueueHandle_t s_queue;

// CRC-16/XMODEM (poly 0x1021, init 0x0000). Bit-by-bit is fine here: this
// runs once per packet (~200Hz) over ~50 bytes, negligible next to the I2C
// and EKF math cost in the same loop.
static uint16_t crc16_xmodem(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t compute_crc(const telemetry_packet_t *pkt)
{
    const uint8_t *bytes = (const uint8_t *)pkt;
    size_t start = offsetof(telemetry_packet_t, seq);
    size_t end = offsetof(telemetry_packet_t, crc);
    return crc16_xmodem(bytes + start, end - start);
}

static void uart_debug_task(void *arg)
{
    telemetry_packet_t pkt;
    while (1) {
        if (xQueueReceive(s_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            // fwrite/fflush go through newlib's stdio lock, so a whole
            // packet is written atomically with respect to other stdout
            // writers (e.g. ESP_LOG from other tasks) -- the two streams
            // interleave at the packet/line level, not mid-write.
            fwrite(&pkt, sizeof(pkt), 1, stdout);
            fflush(stdout);
        }
    }
}

void uart_debug_start(void)
{
    s_queue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(telemetry_packet_t));
    xTaskCreate(uart_debug_task, "uart_debug", UART_DEBUG_TASK_STACK, NULL,
                UART_DEBUG_TASK_PRIORITY, NULL);
}

bool uart_debug_send_telemetry(uint32_t seq, int64_t timestamp_us, const float q[4],
                                const float angular_rate[3], const float gyro_bias[3],
                                const float accel_body[3])
{
    telemetry_packet_t pkt;
    pkt.sync[0] = 0xAA;
    pkt.sync[1] = 0x55;
    pkt.seq = seq;
    pkt.timestamp_us = (uint32_t)timestamp_us;
    memcpy(pkt.q, q, sizeof(pkt.q));
    memcpy(pkt.angular_rate, angular_rate, sizeof(pkt.angular_rate));
    memcpy(pkt.gyro_bias, gyro_bias, sizeof(pkt.gyro_bias));
    memcpy(pkt.accel_body, accel_body, sizeof(pkt.accel_body));
    pkt.crc = compute_crc(&pkt);

    return xQueueSend(s_queue, &pkt, 0) == pdTRUE;
}
