
#include <cuda.h>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>

// /*******************/
// /* iDivUp FUNCTION */
// /*******************/
// //extern "C" int iDivUp(int a, int b){ return ((a % b) != 0) ? (a / b + 1) : (a / b); }
// __host__ __device__ int iDivUp(int a, int b){ return ((a % b) != 0) ? (a / b + 1) : (a / b); }

/********************/
/* CUDA ERROR CHECK */
/********************/
// --- Credit to http://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api
void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{
    if (code != cudaSuccess)
    {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) { exit(code); }
    }
}

extern "C" void gpuErrchk(cudaError_t ans) { gpuAssert((ans), __FILE__, __LINE__); }
    
// Template structure to pass to kernel
template <typename T>
struct KernelArray{
    T*  _array;
    int _size;
};
 
// Function to convert device_vector to structure
template <typename T>
KernelArray<T> convertToKernel(thrust::device_vector<T>& dVec){
    KernelArray<T> kArray;
    kArray._array = thrust::raw_pointer_cast(&dVec[0]);
    kArray._size  = (int) dVec.size();
 
    return kArray;
}

