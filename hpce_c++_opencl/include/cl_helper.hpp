#ifndef CL_HELPER_HPP
#define CL_HELPER_HPP

#define __CL_ENABLE_EXCEPTIONS

#include <cstdio>
#include <fstream>
#include <vector>
#include <stdexcept>
#include "CL/cl.hpp"
#include <map>
#include <iostream>

class CL_helper
{
private:
	std::string LoadSource(const char *fileName) const
	{ 
		std::string baseDir="src/kernels";
		if(getenv("HPCE_CL_SRC_DIR")){
			baseDir=getenv("HPCE_CL_SRC_DIR");
		}

		std::string fullName=baseDir+"/"+fileName;

		// Open a read-only binary stream over the file
		std::ifstream src(fullName, std::ios::in | std::ios::binary);
		if(!src.is_open())
			throw std::runtime_error("LoadSource : Couldn't load cl file from '"+fullName+"'.");

		// Read all characters of the file into a string
		return std::string(
			(std::istreambuf_iterator<char>(src)), // Note the extra brackets.
			std::istreambuf_iterator<char>()
		);
	}

public:
	std::vector<cl::Platform> platforms;
	cl::Platform platform;
	std::vector<cl::Device> devices;
	cl::Device device;

	cl::Context context;
	cl::Program::Sources sources;
	cl::Program::Binaries binaries;
	cl::Program program;
	cl::CommandQueue queue;

	std::vector<cl::Kernel> kernels;

	CL_helper() { }

	CL_helper(const char *kernelFileName, std::vector<std::string> kernelNames, bool loadBinary, bool writeBinary,
				bool usePreProcArgs = false, std::map<std::string, std::string> preProcArgs = std::map<std::string, std::string>()){
		

		cl::Platform::get(&platforms);
		if(platforms.size()==0)
			throw std::runtime_error("No OpenCL platforms found.\n");

 
		int selectedPlatform=0;
		platform=platforms.at(selectedPlatform);
		platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);  
		if(devices.size()==0)
			throw std::runtime_error("No opencl devices found.\n");

		int selectedDevice=0;
		device=devices.at(selectedDevice);
		context = cl::Context(devices);

		

		try{


			if(loadBinary){
				std::string vendor=platform.getInfo<CL_PLATFORM_VENDOR>();
				size_t found = vendor.find("NVIDIA");
				if(found != std::string::npos){
					FILE* fp;
					fp = fopen("src/kernels/julia_filter.ptx", "r");
					if (!fp) {
						std::cerr << "Error loading kernel binary" << std::endl;
						std::cerr << "Building kernel from .cl file" << std::endl;
						loadBinary = false;

					} else {
						fseek(fp, 0, SEEK_END);
						size_t kernel_sz = ftell(fp);
						rewind(fp);

						char* kernel_str = (char*)malloc(kernel_sz);
						unsigned bytes = fread(kernel_str, 1, kernel_sz, fp);
						fclose(fp);

						binaries.push_back(std::make_pair((void*)kernel_str,kernel_sz+1));
				
						program = cl::Program(context, devices, binaries);	

						program.build(devices);
					}
				} else{
					std::cerr << "Vendor not NVIDIA, cannot load .ptx binary" << std::endl;
					std::cerr << "Building kernel from .cl file" << std::endl;
					loadBinary = false;
				}

			}
			if(!loadBinary){
				std::string kernelSource=CL_helper::LoadSource(kernelFileName);
				sources.push_back(std::make_pair(kernelSource.c_str(), kernelSource.size()+1)); 

				program = cl::Program(context, sources);

				if(usePreProcArgs && !preProcArgs.empty()){
					std::string preProcArgsString;
					for(auto& arg : preProcArgs) { 
						preProcArgsString += "-D" + arg.first + "=" + arg.second + " ";
					}
					program.build(devices, preProcArgsString.c_str()); 
				} else {
					//std::string params = "-cl-unsafe-math-optimizations";
					program.build(devices);
				}
			}

		}catch (cl::Error er) {
			for(unsigned i=0;i<devices.size();i++){
					std::cerr <<"Log for device " << devices[i].getInfo<CL_DEVICE_NAME>().c_str()<<std::endl;
					std::cerr << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices[i]).c_str() <<std::endl;
				}
			std::cerr << "ERROR:" << er.what() << " Code " << er.err()<<std::endl;
			throw;
		}


		for(unsigned i=0; i<kernelNames.size();i++){
			kernels.push_back( cl::Kernel(program, kernelNames.at(i).c_str()) );
		}

		queue = cl::CommandQueue(context, device);


		if(writeBinary){
			size_t bin_sz;
			program.getInfo( CL_PROGRAM_BINARY_SIZES, &bin_sz);
			
			unsigned char *bin = (unsigned char *)malloc(bin_sz);
			
			program.getInfo(CL_PROGRAM_BINARIES, &bin);

			FILE* fp = fopen("src/kernels/julia_filter.ptx", "wb");
			fwrite(bin, sizeof(char), bin_sz, fp);
			fclose(fp);
			free(bin);
		}



	}


};


#endif
