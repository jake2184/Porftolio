
#include <cstdio>
#include <stdexcept>
#include <iostream>

#define __CL_ENABLE_EXCEPTIONS

#include "CL/cl.hpp"

int main(int argc, char* argv[])
{
	// Loads add_vectors.cl
	FILE* fp;
	fp = fopen("src/kernels/julia_filter.cl", "r");
	if (!fp) {
		fprintf(stderr, "Error loading kernel.\n");
		exit(1);
	}
 
 	try{

 		std::vector<cl::Platform> platforms;
		cl::Platform platform;
		std::vector<cl::Device> devices;
		cl::Device device;

		cl::Context context;
		cl::Program::Binaries sources;
		cl::Program program;
		cl::CommandQueue queue;

		fseek(fp, 0, SEEK_END);
		size_t kernel_sz = ftell(fp);
		rewind(fp);
	 
		char* kernel_str = (char*)malloc(kernel_sz);
		fread(kernel_str, 1, kernel_sz, fp);
		fclose(fp);
	 
		cl::Platform::get(&platforms);
		if(platforms.size()==0)
			throw std::runtime_error("No OpenCL platforms found.\n");
 
		for(unsigned i=0;i<platforms.size();i++){
	    	std::string vendor=platforms[i].getInfo<CL_PLATFORM_VENDOR>();
		}

		int selectedPlatform=0;
		platform=platforms.at(selectedPlatform);
		platform.getDevices(CL_DEVICE_TYPE_ALL, &devices); 

		if(devices.size()==0)
		    throw std::runtime_error("No opencl devices found.\n");

		for(unsigned i=0;i<devices.size();i++){
		    std::string name=devices[i].getInfo<CL_DEVICE_NAME>();
		}

		int selectedDevice=0;
		device=devices.at(selectedDevice);

		context = cl::Context(devices);

		//std::string kernelSource=CL_helper::LoadSource(kernelFileName);

		//sources.push_back(std::make_pair(kernelSource.c_str(), kernelSource.size()+1)); 
		
		sources.push_back(std::make_pair((void*)kernel_str,kernel_sz+1));

		cl_int err;

		program = cl::Program(context, devices, sources, NULL, &err);
		std::cerr << err << std::endl;


		program.build(devices);

	 
		// Query binary (PTX file) size
		size_t bin_sz;
		err = program.getInfo( CL_PROGRAM_BINARY_SIZES, &bin_sz);
		std::cerr << err << std::endl;

		// // Read binary (PTX file) to memory buffer
		unsigned char *bin = (unsigned char *)malloc(bin_sz);
		
		err = program.getInfo(CL_PROGRAM_BINARIES, &bin);
		std::cerr << err << std::endl;
		std::cerr << bin_sz << std::endl;


		// err = clGetProgramInfo(program, CL_PROGRAM_BINARIES, sizeof(unsigned char *), &bin, NULL);
		// fprintf(stderr, "Binary size: %lu.\n", bin_sz);

		// Save PTX to add_vectors_ocl.ptx
		fp = fopen("julia_filter.ptx", "wb");
		fwrite(bin, sizeof(char), bin_sz, fp);
		fclose(fp);
		free(bin);
	 
	}catch (cl::Error er) {
		std::cerr << "ERROR:" << er.what() << " Code " << er.err()<<std::endl;
		exit(1);
	}catch (...){
		printf("Other error\n");
	}
	return 0;
}