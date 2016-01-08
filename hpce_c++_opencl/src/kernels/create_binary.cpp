#include <CL/cl.h>
#include <cstdio>
using namespace std;
int main(int argc, char* argv[])
{
    // Loads add_vectors.cl
    FILE* fp;
    fp = fopen("julia_filter.cl", "r");
    if (!fp) {
        fprintf(stderr, "Error loading kernel.\n");
        exit(1);
    }
 
    fseek(fp, 0, SEEK_END);
    size_t kernel_sz = ftell(fp);
    rewind(fp);
 
    char* kernel_str = (char*)malloc(kernel_sz);
    fread(kernel_str, 1, kernel_sz, fp);
    fclose(fp);
 
    // Query platforms and devices
    cl_platform_id platform;
    cl_device_id device;
    cl_uint num_devices, num_platforms;
    cl_int err = clGetPlatformIDs(1, &platform, &num_platforms);
    err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 1,
                         &device, &num_devices);
 
    // Create OpenCL context
    cl_context context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
 
    // Create OpenCL command queue
    cl_command_queue command_queue = clCreateCommandQueue(context, device, 0, &err);
 
    // Create OpenCL program 
    cl_program program = clCreateProgramWithSource(context, 1,
            (const char **)&kernel_str, (const size_t *)&kernel_sz, &err);
 
    // Build OpenCL program
    err = clBuildProgram(program, 1, &device, NULL, NULL, NULL);
 
    // Create OpenCL kernel
    //cl_kernel kernel = clCreateKernel(program, "render_fractal", &err);
 
    // Query binary (PTX file) size
    size_t bin_sz;
    err = clGetProgramInfo(program, CL_PROGRAM_BINARY_SIZES, sizeof(size_t), &bin_sz, NULL);

    // Read binary (PTX file) to memory buffer
    unsigned char *bin = (unsigned char *)malloc(bin_sz);
    err = clGetProgramInfo(program, CL_PROGRAM_BINARIES, sizeof(unsigned char *), &bin, NULL);
    fprintf(stderr, "Binary size: %d.\n", bin_sz);

    // Save PTX to add_vectors_ocl.ptx
    fp = fopen("julia_filter.ptx", "wb");
    fwrite(bin, sizeof(char), bin_sz, fp);
    fclose(fp);
    free(bin);
 
    // Release OpenCL resources
    clFlush(command_queue);
    clFinish(command_queue);
    //clReleaseKernel(kernel);
    clReleaseProgram(program);
    clReleaseCommandQueue(command_queue);
    clReleaseContext(context);
    return 0;
}