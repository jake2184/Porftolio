#include "fourier_transform.hpp"

#include <cmath>
#include <cassert>

#include "tbb/parallel_for.h"

namespace hpce
{
namespace jr2812
{
class direct_fourier_transform_chunked : public fourier_transform
{
protected:
	/*! We can do any size transform */
	virtual size_t calc_padded_size(size_t n) const
	{
		return n;
	}

	virtual void forwards_impl(
		size_t n,	const std::complex<double> &/*wn*/,
		const std::complex<double> *pIn, size_t sIn,
		std::complex<double> *pOut, size_t sOut
	) const 
	{
		assert(n>0);
		
		const double PI=3.1415926535897932384626433832795;
		
		// = -i*2*pi / n
		complex_t neg_im_2pi_n=-complex_t(0.0, 1.0)*2.0*PI / (double)n;
		
		unsigned chunkSize;

		char* v = getenv("HPCE_DIRECT_OUTER_K");
		if(v!=NULL){
			chunkSize = atoi(v);
		} else {
			unsigned numCores = sysconf( _SC_NPROCESSORS_ONLN );
			chunkSize = std::max(n / (numCores + numCores ),(long unsigned)1024);
		}
		typedef tbb::blocked_range<unsigned> range_t;

		range_t range(0, n, chunkSize);

		auto f=[=](const range_t &chunk){
			for(unsigned i=chunk.begin(); i!=chunk.end(); i++ ){
        		complex_t acc=0;
				for(size_t ii=0;ii<n;ii++){
					// acc += exp(-i * 2 * pi * kk / n);
					acc+=pIn[ii*sIn] * exp( neg_im_2pi_n * (double)i * (double)ii );
				}
				pOut[i*sOut]=acc;
    		}
		};


		tbb::parallel_for(range, f, tbb::simple_partitioner());
	}
	
	virtual void backwards_impl(
		size_t n,	const std::complex<double> &/*wn*/,
		const std::complex<double> *pIn, size_t sIn,
		std::complex<double> *pOut, size_t sOut
	) const 
	{
		assert(n>0);
		
		const double PI=3.1415926535897932384626433832795;
		
		// = i*2*pi / n
		complex_t im_2pi_n=complex_t(0.0, 1.0)*2.0*PI / (double)n;
		
		const double scale=1.0/n;
		
		int chunkSize;

		char* v = getenv("HPCE_DIRECT_OUTER_K");
		if(v!=NULL){
			chunkSize = atoi(v);
		} else {
			unsigned numCores = sysconf( _SC_NPROCESSORS_ONLN );
			chunkSize = std::max(n / (numCores + numCores ),(long unsigned)1024);
		}

		typedef tbb::blocked_range<unsigned> range_t;

		range_t range(0, n, chunkSize);

		auto f=[=](const range_t &chunk){
			for(unsigned i=chunk.begin(); i!=chunk.end(); i++ ){
        		complex_t acc=0;
				for(size_t ii=0;ii<n;ii++){
					// acc += exp(-i * 2 * pi * kk / n);
					acc+=pIn[ii*sIn] * exp( im_2pi_n * (double)i * (double)ii );
				}
				pOut[i*sOut]=acc*scale;
    		}
		};

		tbb::parallel_for(range, f, tbb::simple_partitioner());
	}
	
public:
	virtual std::string name() const
	{ return "hpce.jr2812.direct_fourier_transform_chunked"; }
	
	virtual bool is_quadratic() const
	{ return true; }
};

std::shared_ptr<fourier_transform> Create_direct_fourier_transform_chunked()
{
	return std::make_shared<direct_fourier_transform_chunked>();
}
}; // namespace jr2812
}; // namespace hpce
