#pragma once

#include <Arduino.h>

class BinBuffer {
    public:
    const uint8_t Nsegments;
    const uint16_t Nsize;
    const uint16_t N;
    volatile unsigned char *buffer;
    volatile uint16_t pos = 0;
    volatile boolean *ready;
    volatile uint32_t error = 0;

    BinBuffer(uint8_t Nsegments, uint16_t Nsize);
    // template <typename T>
    int append(const void *src, size_t len);
};