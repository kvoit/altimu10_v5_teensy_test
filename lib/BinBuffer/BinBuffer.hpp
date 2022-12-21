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

    BinBuffer(uint8_t n_segs, uint16_t seg_size) :
    n_segs(n_segs), seg_size(seg_size), N(n_segs*seg_size) {
        buffer = new volatile unsigned char[N];
        ready = new volatile boolean[n_segs]();
    }

    size_t append(const void *src, size_t len);
    template <typename T> size_t append(const T &src) { return append(&src, sizeof(T)); };
    uint8_t is_ready();
    volatile unsigned char * pop_ready_segment();

    private:
    volatile boolean *ready; // Boolean array to check if a newly started segment is still ready (not yet read)
    std::queue<uint8_t> ready_queue;    // Queue of segment idxs to know which segment needs to be read next 
                                        //(e.g. when segments at the beginning AND end are ready and the latter need to be read first)
};