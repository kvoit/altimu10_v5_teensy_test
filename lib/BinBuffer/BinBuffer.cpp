#include <BinBuffer.hpp>

BinBuffer::BinBuffer(uint8_t n_segs, uint16_t seg_size) :
    n_segs(n_segs), seg_size(seg_size), N(n_segs*seg_size) {
        buffer = new volatile unsigned char[N];
        ready = new volatile boolean[n_segs];

        for(uint8_t i = 0; i<n_segs; i++) {
            ready[i] = false;
        }
    }

// template<typename T>
uint8_t BinBuffer::append(const void *src, size_t len) {
  if(len>seg_size) {// Not prepared for filling a complete segment!
    error |= 1<<1;
    return 1;
  }

  uint8_t old_segment = pos/seg_size;

  const unsigned char *byte_src = reinterpret_cast<const unsigned char*>(src);
  for(uint8_t i = 0; i<len; i++) {
    buffer[(pos+i)%N] = byte_src[i];
  }
  pos = (pos+len)%N;
  
  uint8_t new_segment = pos/seg_size;
  if(old_segment!=new_segment) {
    ready_queue.push(old_segment);

    if(ready[new_segment]) { // We have written into a new segment that was still full!
        error |= 1;
    }
  }
  return 0;
}

uint8_t BinBuffer::is_ready() {
    return ready_queue.size();
}

volatile unsigned char * BinBuffer::pop_ready_segment() {
    uint8_t ready_seg = ready_queue.front();
    ready_queue.pop();
    ready[ready_seg] = false;
    return &buffer[ready_seg*seg_size];
}