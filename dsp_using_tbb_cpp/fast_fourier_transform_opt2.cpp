#include "fourier_transform.hpp"

#include <cmath>
#include <cassert>

#include "tbb/task_group.h"
#include "tbb/parallel_for.h"

namespace hpce
{
namespace jr2812
{
class fast_fourier_transform_opt2 : public fourier_transform
{
protected:
	unsigned HPCE_FFT_RECURSION_K;
	unsigned HPCE_FFT_LOOP_K;
	unsigned numCores;
	/* Standard radix-2 FFT only supports binary power lengths */
	virtual size_t calc_padded_size(size_t n) const
	{
		assert(n!=0);
		
		size_t ret=1;
		while(ret<n){
			ret<<=1;
		}
		
		return ret;
	}

	virtual void forwards_impl(	size_t n, const std::complex<double> &wn,
								const std::complex<double> *pIn, size_t sIn,
								std::complex<double> *pOut, size_t sOut
	) const 
	{
		unsigned chunkSize = std::max(n / (numCores*2), (long unsigned)1024); 
		unsigned recursionLimit = n/16;///numCores;
		my_forwards_impl(n, wn, pIn, sIn, pOut, sOut, chunkSize, recursionLimit);
	}
	virtual void my_forwards_impl(	size_t n, const std::complex<double> &wn,
								const std::complex<double> *pIn, size_t sIn,
								std::complex<double> *pOut, size_t sOut, unsigned chunkSize, 
								unsigned recursionLimit
	) const 
	{
		assert(n>0);
		
		if (n == 1){
			pOut[0] = pIn[0];
		}else if (n == 2){
			pOut[0] = pIn[0]+pIn[sIn];
			pOut[sOut] = pIn[0]-pIn[sIn];		
		}else if (n==4){
			// 4 point FFT
			pOut[0] = pIn[0]+pIn[2*sIn];
			pOut[1] = pIn[0]-pIn[2*sIn];

			pOut[2] = pIn[sIn]+pIn[3*sIn];
			pOut[3] = pIn[sIn]-pIn[3*sIn];

			std::complex<double> w=std::complex<double>(1.0, 0.0);

			std::complex<double> t1 = w*pOut[2];
			std::complex<double> t2 = pOut[0]-t1;
			pOut[0] = pOut[0]+t1;                 
			pOut[2] = t2;                          
			w = w*wn;

			t1 = w*pOut[3];
			t2 = pOut[1]-t1;
			pOut[1] = pOut[1]+t1;                 
			pOut[3] = t2;                          

		}else{
			// parallel task recursion and chunked parallel recombination
			size_t m = n/2;

			//Divide 
			if(n <= HPCE_FFT_RECURSION_K){
				forwards_impl(m,wn*wn,pIn,2*sIn,pOut,sOut);
				forwards_impl(m,wn*wn,pIn+sIn,2*sIn,pOut+sOut*m,sOut);
			}else{
				tbb::task_group group;
				group.run( [=]() {forwards_impl(m,wn*wn,pIn,2*sIn,pOut,sOut);} );
				//group.run( [=]() {forwards_impl(m,wn*wn,pIn+sIn,2*sIn,pOut+sOut*m,sOut);} );
				// Execute on this thread to save thread creation overhead
				forwards_impl(m,wn*wn,pIn+sIn,2*sIn,pOut+sOut*m,sOut);
				group.wait();
			}
/*
	 		// Recombine
			unsigned chunkSize;
			if(sIn == 1){
				if (HPCE_FFT_LOOP_K == 0){
					chunkSize = n / numCores;
				} else {
					chunkSize = HPCE_FFT_LOOP_K;
				}
			}
*/
			//unsigned chunkSize = HPCE_FFT_LOOP_K;
			
			if(m <= chunkSize){
				// Sequential Recombination
				std::complex<double> w=std::complex<double>(1.0, 0.0);
				for (size_t j=0;j<m;j++){
					std::complex<double> t1 = w*pOut[m+j];
					std::complex<double> t2 = pOut[j]-t1;
					pOut[j] = pOut[j]+t1;                 
					pOut[j+m] = t2;                          
					w = w*wn;
				}		
			}else {
				// Parallel Recombination
				typedef tbb::blocked_range<unsigned> range_t;
				range_t range(0, m, chunkSize);

				auto f=[=](const range_t &chunk){
					std::complex<double> w  = pow(wn, chunk.begin());
					for(unsigned j=chunk.begin(); j!=chunk.end(); j++ ){	
							std::complex<double> t1 = w*pOut[m+j];
					  		std::complex<double> t2 = pOut[j]-t1;
					  		pOut[j] = pOut[j]+t1;               
					  		pOut[j+m] = t2;
					  		w = w*wn;  
					}
				};
				tbb::parallel_for(range, f, tbb::simple_partitioner());
			}
		}
	}
	
	virtual void backwards_impl(
		size_t n,	const std::complex<double> &wn,
		const std::complex<double> *pIn, size_t sIn,
		std::complex<double> *pOut, size_t sOut
	) const 
	{
		complex_t reverse_wn=1.0/wn;
		forwards_impl(n, reverse_wn, pIn, sIn, pOut, sOut);
		
		double scale=1.0/n;
		
		// Parfor
		//int numCores = sysconf( _SC_NPROCESSORS_ONLN );

		if(n <= HPCE_FFT_LOOP_K){
			for(size_t i=0;i<n;i++){
			pOut[i]=pOut[i]*scale;
			}
		} else {
			typedef tbb::blocked_range<unsigned> range_t;
			range_t range(0, n, 1024 );

			auto f=[=](const range_t &chunk){
				for(unsigned j=chunk.begin(); j!=chunk.end(); j++ ){	
						pOut[j] = pOut[j] * scale; 
				}
			};

			tbb::parallel_for(range, f);
		}
	}
	
public:
	virtual std::string name() const
	{ return "hpce.jr2812.fast_fourier_transform_opt2"; }
	
	virtual bool is_quadratic() const
	{ return false; }

	fast_fourier_transform_opt2(){
		char *envVar = getenv("HPCE_FFT_RECURSION_K");
		if(envVar != NULL){
			HPCE_FFT_RECURSION_K = atoi(envVar);
		} else {
			HPCE_FFT_RECURSION_K = 32;
		}
		printf("FFT_RECURSION: %d\n", HPCE_FFT_RECURSION_K);
		
		envVar = getenv("HPCE_FFT_LOOP_K");
		if(envVar != NULL){
			HPCE_FFT_LOOP_K = atoi(envVar);
		} else {
			HPCE_FFT_LOOP_K = 1024;
			numCores = sysconf( _SC_NPROCESSORS_ONLN );
		}
		printf("FFT_LOOP: %d\n", HPCE_FFT_LOOP_K);
	};
};

std::shared_ptr<fourier_transform> Create_fast_fourier_transform_opt2()
{
	return std::make_shared<fast_fourier_transform_opt2>();
}

}; // namespace jr2812
}; // namespace hpce

