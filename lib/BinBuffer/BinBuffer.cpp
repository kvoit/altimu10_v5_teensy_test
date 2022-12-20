#include <BinBuffer.hpp>

BinBuffer::BinBuffer(uint8_t Nsegments, uint16_t Nsize) :
    Nsegments(Nsegments), Nsize(Nsize), N(Nsegments*Nsize) {
        buffer = new volatile unsigned char[N];
        ready = new volatile boolean[Nsegments];

        for(uint8_t i = 0; i<Nsegments; i++) {
            ready[i] = false;
        }
    }

// template<typename T>
int BinBuffer::append(const void *src, size_t len) {
  if(len>Nsize) {// Not prepared for filling a complete segment!
    error |= 1<<1;
    return 1;
  }

  uint8_t old_segment = pos/Nsize;

  const unsigned char *byte_src = reinterpret_cast<const unsigned char*>(src);
  for(uint8_t i = 0; i<len; i++) {
    buffer[(pos+i)%N] = byte_src[i];
  }
  pos = (pos+len)%N;
  
  uint8_t new_segment = pos/Nsize;
  if(old_segment!=new_segment) {
    ready[old_segment] = true;

    if(ready[new_segment]) { // We have written into a new segment that was still full!
        error |= 1;
    }
  }
  return 0;
}