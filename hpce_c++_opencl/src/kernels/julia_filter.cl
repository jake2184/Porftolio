#pragma OPENCL EXTENSION cl_khr_fp64: enable
double4 hermite(double x, double4 pix0, double4 pix1, double4 pix2, double4 pix3){
	double4 toRet;

	double xSquare = x*x;
	double xCube = x*x*x;

	double coeff1 = -xCube+2*xSquare-x;
	double coeff2 = 3*xCube-5*xSquare+2;
	double coeff3 = -3*xCube+4*xSquare+x;
	double coeff4 = xCube-xSquare;

	toRet.s0 = pix0.s0*coeff1
			+ pix1.s0*coeff2
			+ pix2.s0*coeff3
			+ pix3.s0*coeff4;
	toRet.s0 /= 2.0;

	toRet.s1 = pix0.s1*coeff1 
			+ pix1.s1*coeff2
			+ pix2.s1*coeff3
			+ pix3.s1*coeff4;
	toRet.s1 /= 2.0;

	toRet.s2 = pix0.s2*coeff1
			+ pix1.s2*coeff2
			+ pix2.s2*coeff3
			+ pix3.s2*coeff4;
	toRet.s2 /= 2.0;

	toRet.s3 = 255;

	return toRet;
}


__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE |
                              CLK_ADDRESS_CLAMP_TO_EDGE |
                              CLK_FILTER_NEAREST;

inline double2 complexMult(double2 first, double2 second){
	return (double2){ 	(first.s0*second.s0 - first.s1*second.s1) ,
						(first.s0*second.s1 + first.s1*second.s0) };
}

__kernel void julia_filter(
	double topLeft,
	__constant double2* c,
	__read_only image2d_t pixles,
	__write_only image2d_t bitmap,
	uint inWidth,
	uint inHeight,
	uint zPow,
	uint maxIter
	){
	
	uint globalY = get_global_id(0);
	uint globalX = get_global_id(1);
	uint height = get_global_size(0);
	uint width = get_global_size(1);


	// Render Fractal
	float fx = (globalX+0.5)/width;
	float fy = (globalY+0.5)/height;

	double2 z;
	z.s0 = topLeft + fx*(-2*topLeft);
	z.s1 = topLeft + fy*(-2*topLeft);	
	double2 closest=z;

 	bool finished = false;
 	double2 valToRet;

 	for(uint i=0; i<maxIter; i++){
		double absZ = z.s0*z.s0 + z.s1*z.s1;
		double absC = closest.s0*closest.s0 + closest.s1*closest.s1;
		if(absZ < absC)
			closest = z;

		if(absZ >= 4){
			valToRet = z;
			i = maxIter+1;
			finished = true;
			break;
		}
		double2 seed = z;
		for(unsigned power=1; power<zPow; power++){
			z = complexMult(z, seed);
		}
		z += c[0];
	}
	if(!finished){
	 	valToRet = closest * 4;
	}

	// Apply fractal to image
	double2 xy;

	xy.s0 = ((valToRet.s0+4)/8) * inWidth;
	xy.s1 = ((valToRet.s1+4)/8) * inHeight;

	int x=floor(xy.s0); 
	int y=floor(xy.s1);

	double dx=xy.s0-x; 
	double dy=xy.s1-y;

	int indexX, indexY;

	double4 rows[4];
	double4 pix[4];

	#pragma unroll
	for(indexY = -1; indexY<=2; indexY++){
		#pragma unroll
		for(indexX = -1; indexX<=2; indexX++){
			uint4 tmp = read_imageui(pixles, sampler, (int2){x+indexX, y+indexY});
		 	rows[indexX+1] = (double4)(tmp.s0, tmp.s1, tmp.s2, tmp.s3);

		}
		pix[indexY+1] = hermite(dx, rows[0],rows[1],rows[2],rows[3]);
	}
	double4 final = hermite(dy, pix[0], pix[1], pix[2], pix[3]);

	uint4 tmp = (uint4)((uint)final.s0, (uint)final.s1, (uint)final.s2, (uint)final.s3);

	write_imageui(bitmap, (int2){globalX, globalY} , tmp);
}



__kernel void julia_filter_no_input(
	double topLeft,
	__constant double2* c,
	__write_only image2d_t bitmap,
	uint zPow,
	uint maxIter
	){

	uint y = get_global_id(0);
	uint x = get_global_id(1);
	uint height = get_global_size(0);
	uint width = get_global_size(1);

	// Render Fractal
	float fx = (x+0.5)/width;
	float fy = (y+0.5)/height;

	double2 z;
	z.s0 = topLeft + fx*(-2*topLeft);
	z.s1 = topLeft + fy*(-2*topLeft);

	double2 closest=z;
 	bool finished = false;

 	uint iterToRet = maxIter;
 	double2 valToRet;

 	for(uint i=0; i<maxIter; i++){

		double absZ = z.s0*z.s0 + z.s1*z.s1;
		double absC = closest.s0*closest.s0 + closest.s1*closest.s1;
		if(absZ < absC)
			closest = z;

		if(absZ >= 4){
			iterToRet = i;
			valToRet = z;
			i = maxIter+1;
			finished = true;
		}
		double2 seed = z;
		for(unsigned power=1; power<zPow; power++){
			z = complexMult(z, seed);
		}
		z += c[0];
	}
	if(!finished){
	 	iterToRet 	= maxIter;
	 	valToRet 	= closest * 4;
	}

	// Convert to image
	unsigned i = iterToRet;
	if(i<maxIter){
		uchar r=(i%7)*36;
		uchar g=((i/7)%7)*36;
		uchar b=((i/49)%7)*36;
		write_imageui(bitmap, (int2){x, y} , (uint4)(r,g,b,255));
	} else{
		double real = valToRet.s0;
		double imag = valToRet.s1;
		uint p = (uint) sqrt( real*real + imag*imag ) * 64;
		unsigned g = min(p, (uint)255);
		write_imageui(bitmap, (int2){x, y} , (uint4)(g,g,g,255));
	}
}
