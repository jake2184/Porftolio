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
#include <chrono>

#include <unistd.h>

#include "reference_jpeg_helpers.hpp"
#include "util.hpp"

//////////////////////////////////////////////////////
// Some basic types

typedef double real_t;

typedef std::complex<real_t> complex_t;

struct pixel_t
{
    real_t r, g, b;
    
    pixel_t()
    {}
    
    pixel_t(real_t _r, real_t _g, real_t _b)
        : r(_r)
        , g(_g)
        , b(_b)
    {}
    
    pixel_t operator+(const pixel_t &o) const
    { return pixel_t(r+o.r,g+o.g,b+o.b); }
    
    pixel_t operator-(const pixel_t &o) const
    { return pixel_t(r-o.r,g-o.g,b-o.b); }
    
    pixel_t operator-() const
    { return pixel_t(-r,-g,-b); }
    
    pixel_t operator*(real_t p) const
    { return pixel_t(r*p,g*p,b*p); }
    
    pixel_t operator/(real_t p) const
    { return pixel_t(r/p,g/p,b/p); }
    
    unsigned clamp(unsigned x) const
    { return std::max(0u,std::min(255u,x)); }
    
    uint32_t to_rgb24() const
    {
        return clamp(b*255)
            +clamp(g*255)*256
            +clamp(r*255)*65536;
    }
};


///////////////////////////////////////////////////////////////
// Field class, maps complex co-ordinates onto a lattice
// and supports sampling within the field at any complex coord

template<class TElement>
struct field_t
{
    unsigned width;
    unsigned height;
    complex_t topLeft, bottomRight;
    std::vector<TElement> field;
    
    field_t(unsigned w, unsigned h, complex_t tl, complex_t br)
        : width(w)
        , height(h)
        , topLeft(tl)
        , bottomRight(br)
        , field(w*h)
    {}
    
    void set_element(int x, int y, const TElement &value)
    {
        field[y*width+x]=value;
    }
    
    TElement get_element(int x, int y) const
    {
        x=std::max(0,std::min((int)width-1,x));
        y=std::max(0,std::min((int)height-1,y));
        return field[y*width+x];
    }
   
    /*
    TElement sample_at_coord(const complex_t &coord) const
    {
        complex_t xy=complex_t(
            (coord.real()-topLeft.real()) / (bottomRight.real()-topLeft.real()) * width,
            (coord.imag()-topLeft.imag()) / (bottomRight.imag()-topLeft.imag()) * height
        );
        auto v00=get_element((int)floor(xy.real()), (int)floor(xy.imag()));
        auto v01=get_element((int)floor(xy.real()), (int)ceil(xy.imag()));
        auto v10=get_element((int)ceil(xy.real()), (int)floor(xy.imag()));
        auto v11=get_element((int)ceil(xy.real()), (int)ceil(xy.imag()));
        real_t h=xy.real()-floor(xy.real());
        real_t v=xy.imag()-floor(xy.imag());
                
        return v00*(1-h)*(1-v)+v01*(1-h)*v+v10*h*(1-v)+v11*h*v;
    }
    
    */
    
    
    // https://en.wikipedia.org/wiki/Cubic_Hermite_spline
    TElement hermite(real_t x, TElement a, TElement b, TElement c, TElement d) const
    {
        return(
              a*(-x*x*x+2*x*x-x)
            + b*(3*x*x*x-5*x*x+2)
            + c*(-3*x*x*x+4*x*x+x)
            + d*(x*x*x-x*x)
        ) / 2;
    }
    
    /*
        Bicubic can look worse on synthetic images, as it tends to
        ring at very hard edges. Looks better on natural though.
    */
    TElement sample_at_coord(const complex_t &coord) const
    {

	std::cerr << coord.real() << " " << coord.imag() << std::endl;
        complex_t xy=complex_t(
            (coord.real()-topLeft.real()) / (bottomRight.real()-topLeft.real()) * width,
            (coord.imag()-topLeft.imag()) / (bottomRight.imag()-topLeft.imag()) * height
        );
        int x=floor(xy.real()), y=floor(xy.imag());
        real_t dx=xy.real()-x;
        real_t dy=xy.imag()-y;
        
        assert(dx>=0);
        assert(dx<=1);
        
        auto at = [&](int x, int y)
        { return get_element(x,y); };
        
        return hermite(
            dy,
            hermite(dx, at(x-1,y-1), at(x+0,y-1), at(x+1,y-1), at(x+2,y-1)),
            hermite(dx, at(x-1,y+0), at(x+0,y+0), at(x+1,y+0), at(x+2,y+0)),
            hermite(dx, at(x-1,y+1), at(x+0,y+1), at(x+1,y+1), at(x+2,y+1)),
            hermite(dx, at(x-1,y+2), at(x+0,y+2), at(x+1,y+2), at(x+2,y+2))
        );
    }
    
    
    complex_t get_element_coord(int x, int y) const
    {
        real_t fx=(x+0.5)/width;
        real_t fy=(y+0.5)/height;
        return complex_t(
            topLeft.real()+fx*(bottomRight.real()-topLeft.real()),
            topLeft.imag()+fy*(bottomRight.imag()-topLeft.imag())
        );
    }
};


////////////////////////////////////////////////////////
// Methods fors working with fields

template<class TFunc, class TElement>
void generate_field(
    TFunc f,
    field_t<TElement> &dst
){
    for(unsigned x=0; x<dst.width; x++){
        for(unsigned y=0; y<dst.height; y++){
            auto coord=dst.get_element_coord(x,y);
            auto value=f(coord);
            dst.set_element(x,y,value);
        }
    }
}

template<class TSrcElement, class TFunc, class TDstElement>
void map_field(
    const field_t<TSrcElement> &src,
    TFunc f,
    field_t<TDstElement> &dst
){
    for(unsigned x=0; x<dst.width; x++){
        for(unsigned y=0; y<dst.height; y++){
            complex_t dstCoord=dst.get_element_coord(x,y);
            complex_t srcValue=src.sample_at_coord(x,y);
            dst.set_element(x,y,f(srcValue));
        }
    }
}

template<class TElement, class TFunc>
void bitmap_to_field(
    std::vector<uint8_t> src,
    TFunc rgb24_to_element,
    field_t<TElement> &dst
){
    unsigned w=dst.width, h=dst.height;
    unsigned stride=((w*3+3)/4)*4;
    
    for(unsigned y=0; y<h; y++){
		for(unsigned x=0;x<w;x++){
            uint32_t rgb24=
                uint32_t(src[y*stride+x*3])+
                (uint32_t(src[y*stride+x*3+1])<<8)+
                (uint32_t(src[y*stride+x*3+2])<<16);

            TElement value=rgb24_to_element(rgb24);
            
            dst.set_element(x,y,value);
        }
    }
}


template<class TElement, class TFunc>
std::vector<uint8_t> field_to_bitmap(
    const field_t<TElement> &src,
    TFunc element_to_rgb32
){
    unsigned w=src.width, h=src.height;
    unsigned stride=((w*3+3)/4)*4;
    
    std::vector<uint8_t> res;
    
    for(unsigned y=0; y<h; y++){
		for(unsigned x=0;x<w;x++){
            uint32_t rgb32=element_to_rgb32(src.get_element(x,y));
            
            res.push_back( rgb32&0xFF );
            res.push_back( (rgb32>>8)&0xFF );
            res.push_back( (rgb32>>16)&0xFF );
        }
        int pad=stride-w*3;
        for(int i=0;i<pad;i++){
            res.push_back(0);
        }
    }
    
    return res;
}

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

int main(int argc, char *argv[])
{
    auto startTime = std::chrono::high_resolution_clock::now();
    try{
        int outWidth=640, outHeight=480;
        int maxIter=256;
        int jpegQuality=95;
        int maxFrames=INT_MAX;
        double targetFrameRate=25;
        double zPow=2.0;
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
                    fprintf(stderr, "No number following --target-frame-rate.\n");
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
                zPow=strtod(argv[ai+1],NULL);
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
        
        ////////////////////////////////////////////////////
        // Main frame loop
        double t=animTStart;
        int frames=0;
        double firstFrame;
        int onTimeFrames=0;
	auto loopStartTime = std::chrono::high_resolution_clock::now();
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
            // Convert the input image to a field of pixels
            field_t<pixel_t> in(inWidth,inHeight,complex_t(-4,-4),complex_t(+4,+4));
            
            bitmap_to_field(
                inPixels,
                [=](uint32_t pixel){
                    return pixel_t( ((pixel>>16)&0xFF)/256.0, ((pixel>>8)&0xFF)/256.0, (pixel&0xFF)/256.0 );
                },
                in
            );                  
                
            //////////////////////////////////////////////////////////////
            // Do some animation of parameters
            double zoomScale=(2+sin(t*animZoomScale))/3;
            field_t<std::pair<real_t,complex_t> > field(outWidth,outHeight,complex_t(-2*zoomScale,-2*zoomScale),complex_t(+2*zoomScale,+2*zoomScale));
            
            double cScale=(2+sin(t*animCScale))/3;
            
            complex_t c(sin(t)*cScale,cos(t)*cScale);
            real_t escape=2;                
            
            ///////////////////////////////////////////////////////////////
            // Render the actual fractal                
            auto iter = [=](complex_t zInit){
                complex_t z=zInit;
                complex_t closest=zInit;
                for(int i=0; i<maxIter; i++){
                    if(std::abs(z) < std::abs(closest)){
                        closest=z;
                    }
                    if(std::abs(z) >= escape){
                        return std::make_pair((real_t)i,z);
                    }
                    z=pow(z,zPow)+c;
                }
                return std::make_pair((real_t)maxIter, closest*real_t(4));
            };
            generate_field(iter, field);            
            
            
            //////////////////////////////////////////////////////////////
            // Convert the fractal to a bitmap
            auto color = [=](std::pair<real_t,complex_t> elt)
            {
                auto clamp=[](unsigned x)
                { return std::max(0u,std::min(255u,x)); };
                
                if(noInput){
                    unsigned i=elt.first;
                    if(i<maxIter){
                        unsigned r=(i%7)*36;
                        unsigned g=((i/7)%7)*36;
                        unsigned b=((i/49)%7)*36;
                        return clamp(r) + (clamp(g)<<8) + (clamp(b)<<16);
                    }else{
                        real_t x=std::abs(elt.second)*64;
                        unsigned g=std::min((unsigned)x,255u);
                        return clamp(g) + (clamp(g)<<8) + (clamp(g)<<16);
                    }
                }else{
                    return in.sample_at_coord(elt.second).to_rgb24();
                }
            };std::vector<uint8_t> bitmap=field_to_bitmap(
                field,
                color
            );
                
            ///////////////////////////////////////////////////////////////
            // Send the frame to stdout
            if(rawOutput){
                if(1!=fwrite(&bitmap[0], bitmap.size(), 1, outFile)){
                    std::cerr<<"Couldn't write rgb24 image to stdout.\n";
                    exit(1);
                }
            }else{
                write_JPEG_file (field.width, field.height, bitmap, outFile, jpegQuality);
            }
            
            t=t+animTScale;
            frames++;
        }
        auto loopEndTime = std::chrono::high_resolution_clock::now();
	double loopTime = std::chrono::duration_cast<std::chrono::duration<double>>(loopEndTime - loopStartTime).count();
 	std::cerr << "Loop Execution Time: " << loopTime << ", Frames: " << frames << ", Avg. Loop Time: " << double(loopTime / frames) << std::endl;
        
        if(inFile!=stdin){
            fclose(inFile);
        }
        if(outFile!=stdout){
            fclose(outFile);
        }
    }catch(std::exception &e){
        std::cerr<<"Exception : "<<e.what()<<"\n";
        exit(1);
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    std::cerr << "Execution time: " << std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count() << std::endl;    
    return 0;
}
