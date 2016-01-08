#include <complex>
#include <vector>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <cstdlib>
#include <climits>
#include <cstring>
#include <cassert>
#include <thread>
#include <atomic>

#define __CL_ENABLE_EXCEPTIONS

#include <unistd.h>

#include "jpeg_helpers.hpp"
#include "util.hpp"
#include "cl_helper.hpp"
#include "CL/cl.h"
#include <iomanip>
//////////////////////////////////////////////////////
// Some basic types

typedef double real_t;

typedef std::complex<real_t> complex_t;

////////////////////////////////////////////////////////
// Driver program

void print_usage()
{
	std::cerr<<"Usage: julia_filter\n";
	std::cerr<<"  --target-frame-rate real : Frame-rate desired by user\n";
	std::cerr<<"  --jpeg-quality int : Encoding quality 0..100\n";
	std::cerr<<"  --max-frames int : Stop after this many output frames (default=INT_MAX)\n";
	std::cerr<<"  --width int : Output image width\n";
	std::cerr<<"  --height int : Output image height\n";
	std::cerr<<"\n";
	std::cerr<<"  --input-file file : Set source file (default = stdin)\n";
	std::cerr<<"  --ouput-file file : Set destination file (default = stdout)\n";
	std::cerr<<"  --no-input : Specify that there is not an input file\n";
	std::cerr<<"  --rgb24-input : Input is word-aligned rgb24, not jpeg\n";
	std::cerr<<"  --rgb24-output : Output is word-aligned rgb24, not jpeg\n";
	std::cerr<<"\n";
	std::cerr<<"  --max-iter int : Iteration depth before bailout\n";
	std::cerr<<"  --zpow real : Exponent e to use in z=z^e+c (default=2)\n";
	std::cerr<<"  --anim-t-start real : Start time for animation (default=0)\n";
	std::cerr<<"  --anim-t-scale real : Time per frame for animation (default=0.01)\n";
	std::cerr<<"  --anim-c-scale real : Speed for c scaling (default=0.5)\n";
	std::cerr<<"  --anim-zoom-scale real : Speed for zoom scaling (default=7)\n";
	std::cerr<<"\n";
	std::cerr<<"  --aesthetic : Enable aesthetic mode\n";
}

void writeBehind(int outWidth, int outHeight, int jpegQuality, FILE *outFile, int frameNo, std::vector<uint8_t> *bitmap0, std::vector<uint8_t> *bitmap1, std::atomic<int>* frameCounter) {
	int current = 0;
	while(current < frameNo) {
		if(*frameCounter >= 1){
			write_JPEG_file(outWidth, outHeight, (current & 0x1 ? *bitmap1 : *bitmap0), outFile, jpegQuality);
			*frameCounter = *frameCounter - 1;
			current++;
		}
	}
}


void writeBehindRaw(int outWidth, int outHeight, int jpegQuality, FILE *outFile, int frameNo, std::vector<uint8_t> *bitmap0, std::vector<uint8_t> *bitmap1, std::atomic<int>* frameCounter) {
	int current = 0;
	while(current < frameNo) {
		if(*frameCounter >= 1){
			unsigned pixelsToWrite = outWidth * outHeight;
			std::vector<uint8_t>* bitmap = (current & 0x1 ? bitmap1 : bitmap0);
			std::vector<uint8_t> outMap(pixelsToWrite * 3);
			std::vector<uint8_t>::iterator placeIt = outMap.begin();
			std::vector<uint8_t>::iterator fromIt = (*bitmap).begin();
			std::vector<uint8_t>::iterator endIt = fromIt + 3;
			for(int i = 0; i < pixelsToWrite; i++) {
				std::move(fromIt, endIt, placeIt);
				placeIt += 3;
				fromIt += 4;
				endIt = fromIt + 3;
			}
			if(1!=fwrite(  &outMap[0] , outMap.size(), 1, outFile)){
				std::cerr<<"Couldn't write rgb24 image to stdout.\n";
				exit(1);
			}
			*frameCounter = *frameCounter - 1;
			current++;
		}
	}
}

int main(int argc, char *argv[])
{
	try{
		int outWidth=640, outHeight=480;
		int maxIter=256;
		int jpegQuality=95;
		int maxFrames=INT_MAX; //?
		double targetFrameRate=25;
		int zPow=2;
		bool noInput=false, rawInput=false, rawOutput=false;
		FILE *inFile=stdin, *outFile=stdout;
		bool aesthetic=false;

		double animTStart=0, animTScale=0.01;
		double animZoomScale=0.5, animCScale=7;

		///////////////////////////////////////////////////////////////////////
		// All options
		int ai=1;
		while(ai<argc){
			if(!strcmp("--target-frame-rate",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --max-frames.\n");
					exit(1);
				}
				targetFrameRate=strtod(argv[ai+1], NULL);
				ai+=2;
			}else if(!strcmp("--jpeg-quality",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --jpeg-quality.\n");
					exit(1);
				}
				jpegQuality=atoi(argv[ai+1]);
				ai+=2;
			}else if(!strcmp("--width",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --width.\n");
					exit(1);
				}
				outWidth=atoi(argv[ai+1]);
				ai+=2;
			}else if(!strcmp("--height",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --height.\n");
					exit(1);
				}
				outHeight=atoi(argv[ai+1]);
				ai+=2;
			}else if(!strcmp("--max-iter",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --max-iter.\n");
					exit(1);
				}
				maxIter=atoi(argv[ai+1]);
				ai+=2;
			}else if(!strcmp("--max-frames",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --max-frames.\n");
					exit(1);
				}
				maxFrames=atoi(argv[ai+1]);
				ai+=2;
			}else if(!strcmp("--zpow",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --zpow.\n");
					exit(1);
				}
				zPow=atoi(argv[ai+1]);
				ai+=2;
			}else if(!strcmp("--no-input",argv[ai])){
				noInput=true;
				ai++;
			}else if(!strcmp("--input-file",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No file-name following --input-file.\n");
					exit(1);
				}
				std::cerr<<"Opening input file '"<<argv[ai+1]<<"\n";
				FILE *tmp=fopen(argv[ai+1], "rb");
				if(!tmp){
					fprintf(stderr, "Couldn't open input file '%s'\n", argv[ai+1]);
					exit(1);
				}
				inFile=tmp;
				ai+=2;
			}else if(!strcmp("--output-file",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No file-name following --output-file.\n");
					exit(1);
				}
				std::cerr<<"Opening output file '"<<argv[ai+1]<<"\n";
				FILE *tmp=fopen(argv[ai+1], "wb");
				if(!tmp){
					fprintf(stderr, "Couldn't open output file '%s'\n", argv[ai+1]);
					exit(1);
				}
				outFile=tmp;
				ai+=2;
			}else if(!strcmp("--rgb24-input",argv[ai])){
				rawInput=true;
				ai++;
			}else if(!strcmp("--rgb24-output",argv[ai])){
				rawOutput=true;
				ai++;
			}else if(!strcmp("--anim-t-start",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --anim-t-start.\n");
					exit(1);
				}
				animTStart=strtod(argv[ai+1],NULL);
				ai+=2;
			}else if(!strcmp("--anim-t-scale",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --anim-t-scale.\n");
					exit(1);
				}
				animTScale=strtod(argv[ai+1],NULL);
				ai+=2;
			}else if(!strcmp("--anim-zoom-scale",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --anim-zoom-scale.\n");
					exit(1);
				}
				animZoomScale=strtod(argv[ai+1],NULL);
				ai+=2;
			}else if(!strcmp("--anim-c-scale",argv[ai])){
				if(ai+1==argc){
					fprintf(stderr, "No number following --anim-c-scale.\n");
					exit(1);
				}
				animCScale=strtod(argv[ai+1],NULL);
				ai+=2;
			}else if(!strcmp("--aesthetic",argv[ai])){
				aesthetic=true;
				ai+=2;
			}else{
				fprintf(stderr, "Unknown option '%s'\n", argv[ai]);
				exit(1);
			}
		}

		std::cerr<<"Info: width="<<outWidth<<", height="<<outHeight<<", maxIter="<<maxIter<<", maxFrames="<<maxFrames<<"\n";
		std::cerr<<"      jpegQuality="<<jpegQuality<<", targetFrameRate="<<targetFrameRate<<"\n";

		read_JPEG_file jpeg_src(inFile);
		bool gotImage=false, finalImage=false;

		///////////////////////////////////////
		// Initialise GPU

		// Initialise constants
		unsigned imageSize = outWidth * outHeight;

		// Initialise host buffers
		std::vector<uint8_t>bitmap0(4*imageSize);
		std::vector<uint8_t>bitmap1(4*imageSize);

		// Initialise CL helper 
		std::vector<std::string> kernelNames;
		kernelNames.push_back("julia_filter");
		kernelNames.push_back("julia_filter_no_input");
		CL_helper gpu("julia_filter.cl", kernelNames, true, false);


		// Initialise GPU buffers
		cl::Buffer cin(gpu.context, CL_MEM_READ_ONLY, 8*2);

		cl_int error;
		cl::ImageFormat format= {CL_RGBA, CL_UNSIGNED_INT8};
		cl::Image2D bitmapImage(gpu.context, CL_MEM_WRITE_ONLY, format, outWidth, outHeight);

		cl::NDRange offset(0,0);
		cl::NDRange globalSize(outHeight, outWidth);
		cl::NDRange localSize=cl::NullRange;

		cl::size_t<3> outOrigin;
		outOrigin.push_back(0); outOrigin.push_back(0); outOrigin.push_back(0);
		cl::size_t<3> outRegion;
		outRegion.push_back(outWidth);outRegion.push_back(outHeight);outRegion.push_back(1);

		////////////////////////////////////////////////////
		// Main frame loop
		double t=animTStart;
		int frames=0;
		double firstFrame;
		int onTimeFrames=0;

		std::atomic<int> frameCounter (0);
		std::thread t1((rawOutput ? writeBehindRaw : writeBehind), outWidth, outHeight, jpegQuality, outFile, maxFrames, &bitmap0, &bitmap1, &frameCounter);
		while(frames<maxFrames){
			std::vector<uint8_t> inPixels;
			int inWidth=0, inHeight=0;
			////////////////////////////////////////////////////////////
			// Try to read next input image.
			if(!(noInput||finalImage)){
				std::vector<uint8_t> tmpPixels;
				int tmpWidth, tmpHeight;

				if(jpeg_src.read(tmpWidth, tmpHeight, tmpPixels)){
					if(!gotImage){
						std::cerr<<"No source image, using lookup.\n";
						noInput=true;
					}else{
						std::cerr<<"Final source image, staying static.\n";
						finalImage=true;
					}
				}else{
					std::swap(inWidth, tmpWidth);
					std::swap(inHeight, tmpHeight);
					std::swap(inPixels, tmpPixels);
					gotImage=true;
				}
			}

			//////////////////////////////////////////////////////////////
			// Do some animation of parameters

			double zoomScale=(2+sin(t*animZoomScale))/3;
			double fieldTopLeft = -2*zoomScale;
			double cScale=(2+sin(t*animCScale))/3;
			complex_t c(sin(t)*cScale,cos(t)*cScale); 

			cl_double ci[2] = {c.real(), c.imag()};
			gpu.queue.enqueueWriteBuffer(cin, CL_TRUE, 0, 8*2, &ci[0]);

			if(noInput){
				gpu.kernels[1].setArg(0, fieldTopLeft);
				gpu.kernels[1].setArg(1, cin);
				gpu.kernels[1].setArg(2, bitmapImage);
				gpu.kernels[1].setArg(3, zPow);
				gpu.kernels[1].setArg(4, maxIter);

				gpu.queue.enqueueNDRangeKernel(gpu.kernels[1], offset, globalSize, localSize);
			} else{
				cl::Image2D pixelImage(gpu.context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, format , 
							inWidth, inHeight, 0, &inPixels[0], &error);
				gpu.kernels[0].setArg(0, fieldTopLeft);
				gpu.kernels[0].setArg(1, cin);
				gpu.kernels[0].setArg(2, pixelImage);
				gpu.kernels[0].setArg(3, bitmapImage);
				gpu.kernels[0].setArg(4, inWidth);
				gpu.kernels[0].setArg(5, inHeight);
				gpu.kernels[0].setArg(6, zPow);
				gpu.kernels[0].setArg(7, maxIter);

				gpu.queue.enqueueNDRangeKernel(gpu.kernels[0], offset, globalSize, localSize);

			}

			gpu.queue.enqueueBarrier();

			while(frameCounter >= 2){
				sleep(0.0001);
			}

			gpu.queue.enqueueReadImage(bitmapImage, CL_TRUE, outOrigin, outRegion, 0,0, (frames & 0x1 ? &bitmap1[0] : &bitmap0[0]));
			frameCounter = frameCounter + 1;

			t=t+animTScale;
			frames++;
		}

		t1.join();

		if(inFile!=stdin){
			fclose(inFile);
		}
		if(outFile!=stdout){
			fclose(outFile);
		}
	}catch(cl::Error er){
		std::cerr << "ERROR: " << er.what() << " Code " << er.err() <<std::endl;
		exit(1);
	}catch(std::exception &e){
		std::cerr<<"Exception : "<<e.what()<<"\n";
		exit(1);
	}
	return 0;
}
