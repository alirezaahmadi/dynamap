#include "heap.h"

namespace DynaMap {

namespace allocMem {

void Heap::Init(int heap_size) {
  cudaMallocManaged(&heap_, sizeof(unsigned int) * heap_size);
  cudaDeviceSynchronize();
}

// todo .. deconstructor of allocMem ??  to free heap

__device__ unsigned int Heap::Consume() {
  unsigned int idx = atomicSub(&heap_counter_, 1);
  return heap_[idx];
}

__device__ void Heap::Append(unsigned int ptr) {
  unsigned int idx = atomicAdd(&heap_counter_, 1);
  heap_[idx + 1] = ptr;
}


}  // namespace allocMem

}  // namespace DynaMap
