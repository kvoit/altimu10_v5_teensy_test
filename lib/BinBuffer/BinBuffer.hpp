#pragma once

#include <Arduino.h>
#include <queue>

class BinBuffer {
    public:
    const uint8_t n_segs;
    const uint16_t seg_size;
    const uint16_t N;
    volatile unsigned char *buffer;
    volatile uint16_t pos = 0;
    volatile uint32_t error = 0;

    BinBuffer(uint8_t n_segs, uint16_t seg_size);
    // template <typename T>
    uint8_t append(const void *src, size_t len);
    uint8_t is_ready();
    volatile unsigned char * pop_ready_segment();

    private:
    volatile boolean *ready;
    std::queue<uint8_t> ready_queue;
};