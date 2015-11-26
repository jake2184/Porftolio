#include "fourier_transform.hpp"

#include <cmath>
#include <cassert>

#include "tbb/task_group.h"
#include "tbb/parallel_for.h"

namespace hpce
{
namespace jr2812
{
class fast_fourier_transform_combined : public fourier_transform
{
protected:
	unsigned recursionLimit;
	unsigned HPCE_FFT_LOOP_K;
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
		assert(n>0);
		
		if (n == 1){
			pOut[0] = pIn[0];
		}else if (n == 2){
			pOut[0] = pIn[0]+pIn[sIn];
			pOut[sOut] = pIn[0]-pIn[sIn];
		}else if (n <= recursionLimit || n/2 <= HPCE_FFT_LOOP_K){
			// serial recursion execution
			size_t m = n/2;

			forwards_impl(m,wn*wn,pIn,2*sIn,pOut,sOut);
			forwards_impl(m,wn*wn,pIn+sIn,2*sIn,pOut+sOut*m,sOut);
			 
			std::complex<double> w=std::complex<double>(1.0, 0.0);

			for (size_t j=0;j<m;j++){	
			  std::complex<double> t1 = w*pOut[m+j];
			  std::complex<double> t2 = pOut[j]-t1;
			  pOut[j] = pOut[j]+t1;                 
			  pOut[j+m] = t2;                          
			  w = w*wn;
			}
		}else{
			// parallel task recursion
			size_t m = n/2;

			tbb::task_group group;

			group.run( [=]() {forwards_impl(m,wn*wn,pIn,2*sIn,pOut,sOut);} );
			group.run( [=]() {forwards_impl(m,wn*wn,pIn+sIn,2*sIn,pOut+sOut*m,sOut);} );

			group.wait();

	 
			typedef tbb::blocked_range<unsigned> range_t;

			range_t range(0, m, HPCE_FFT_LOOP_K);

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
	
	virtual void backwards_impl(
		size_t n,	const std::complex<double> &wn,
		const std::complex<double> *pIn, size_t sIn,
		std::complex<double> *pOut, size_t sOut
	) const 
	{
		complex_t reverse_wn=1.0/wn;
		forwards_impl(n, reverse_wn, pIn, sIn, pOut, sOut);
		
		double scale=1.0/n;
		for(size_t i=0;i<n;i++){
			pOut[i]=pOut[i]*scale;
		}
	}
	
public:
	virtual std::string name() const
	{ return "hpce.jr2812.fast_fourier_transform_combined"; }
	
	virtual bool is_quadratic() const
	{ return false; }

	fast_fourier_transform_combined(){
		char *envVar = getenv("HPCE_FFT_RECURSION_K");
		if(envVar != NULL){
			recursionLimit = atoi(envVar);
		} else {
			recursionLimit = 1024;
		}
		
		envVar = getenv("HPCE_FFT_LOOP_K");
		if(envVar != NULL){
			HPCE_FFT_LOOP_K = atoi(envVar);
		} else {
			HPCE_FFT_LOOP_K = 1024 ;
		}
	};
};

std::shared_ptr<fourier_transform> Create_fast_fourier_transform_combined()
{
	return std::make_shared<fast_fourier_transform_combined>();
}

}; // namespace jr2812
}; // namespace hpce
