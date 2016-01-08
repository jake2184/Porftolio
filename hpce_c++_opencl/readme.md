HPCE 2015 CW6 CYL12/JR2812
==========================

Submission for HPCE CW6, Julia Filter.
Majority of work was completed on DoC graphic PCs, with a powerful NVIDIA Quadro 6000 GPU. Any comparisons of speed below are based on this. Considerations for hardware are described in Future Optimisations/Known Slow points.

Work Split
----------
It was a mixture of individual work to get specific aspects working and sitting around one computer trying to identify the next to optimize. Jake spent more time with GPU implementations as he is very good with them now and seems to find tearing his hair out, trying to debug obscure GPU problems, enjoyable. Cian spent more time profiling the application and adding the file input/output optimizations. While we both worked on each others aspects, we were more focused on the fractal calculations (Jake) and file interface (Cian).

Both of us aren't very artistic and we were more interested in the technical implementations and speed ups that we could achieve, so decided not to do the aesthetic mode. 

Dependencies
------------
sudo apt-get -y install curl autoconf automake libtool nasm

Building
--------
As expected

Running
-------
As expected  

IMPROVEMENTS
============

Structural
----------
- The field structures are unnecessary overhead, so removed

OpenCL
------
- The sequential generation of the fractal, and application to the input (or processing for no-input) is slow
- Perfectly suited for a GPU, as a function is exectued for every pixel. So we moved to OpenCL on GPU
- Kernels for generating the fractal and applying it to no-input/input are combined to reduce buffer usage
- Kernels have been roughly optimised, but not to great lengths, as the bottle-neck is file writing (see below)
- An example of this is the use of doubles. Converting all double{2|4} to float{2|4} is 1.4x faster on the kernel, with no obvious visual degredation. However, total execution time is not affected, so doubles were left in to ensure higher quality output. This was tested with git tags v1.4/v1.5_float
- The use of images rather than global buffers reduces kernel time due the way images are handled/stored on the GPU, as well as providing inbuilt address clamping. 
- We attempt to load the kernels from a .ptx binary. If the binary is not found, or the OpenCL device vendor is not NVIDIA, we default to reading the kernel from a .cl file and build it. The speed improvement is difficult to measure, as compiled kernels appear to be cached between runs, but from a cold start it can make a large difference. Compilation can take up to 1 second, and given that this is the time taken to process/write ~140 frame video (DoC Graphic04), it is a huge overhead. 

JPEG file handling
---------------
- File reading and writing is done with libjpeg-turbo, to allow the introduction of an alpha channel, with values we can safely ignore and not concern ourselves with. This allows us to use images with the required format CL_RGBA for OpenCL.
- The slowest part of the program, post putting arithmetic on the GPU as above, is writing to file. write_JPEG is the critial path. This is primarily due to the slow Hoffman encoding rather than actual writing to file
- We spin off the file writing into a separate thread, with a shared buffer (size 2 frames) which the primary thread reads the generated bitamp from the GPU into
- As long as the GPU can create frames more quickly than it takes to compress and write one, which it did in all of our tests, then write_JPEG will be running constantly (with certain assumptions, eg primary process and CPU cores >= 2)
- write_JPEG is therefore the critical path. Hard to reduce overall runtime further than this. 
- Investigated how to speed write_JPEG up. Increasing the scan_lines compressed at once slows down the process, and other alterations require editing the library (and likely therefore to not exist/be worthwhile)  

Future Optimisations/Known Slow points
--------------------------------------
- We read and write an unused alpha channel to the GPU and back. This is 1/3 extra data, and time for data transfers host<->device seem to be ~linear relative to size.
- Kernels have a lot of divergent flow control. Inspecting generated ptx code shows this is not ideal
- Our comparisons were done a DoC graphic computer, with a very powerful GPU. It is quite possible that on a device with a weaker GPU but still a decent CPU (Jake's machine before breaking was a good example of this) that frames cannot be generated as fast as they are written. Our program will still function perfectly, but the critical path changes. The impact of double arithmetic may also have more of an effect. 
- It is difficult to compare/test the above, as it is hard to see the true time difference between the GPU thread and the writing thread. This is because the GPU thread blocks to allow the write to catch up with more than 1 unwritten output frame. The effect could be minimised by increasing the size of our buffer, however this wouldn't affect execution time, and would increase our memory usage. 
- An interesting approach to reduce write time is by using tbb to execute write_JPEG in parallel. This could be done with tasks creating a file for every frame, which are then combined at the end with ffmpeg into an mpeg, or could possibly be done with tbb::pipeline. 
- In the same manner as the writing is done, reading of the file could be spun off into another thread. However, the read time is fairly small compared to the time for GPU/writing, and creation of another thread reduces effectiveness eg on a dual-core machine. 

Benchmarking / Critical Path Analysis
-------------------------------------
- We determined the critical path by analyzing bin/julia_filter with the perf tool on the graphic machines. They highlighted that the encode_mcu_huff method within libjpeg was taking the majority of the execution time (30+%) which meant that we aimed to optimize the binary so that writing became the critical path and latency could be reduced to (almost) the minimum. 
- Apart from using libjpeg-turbo which provides faster encoding/decoding, we also created the writing thread as discussed above.
- To show the approximate critical path and relation to the writing thread, we ran some tests to determine the FPS of each section of the julia_filter. Results are below for 100 frames no-input, rgb24-output:
  - **Write**
    - **Total Time:** 0.296184
    - **Avg. Time:** 0.00296184
    - **FPS:** 337.628
  - **Read**
    - **Total Time:** 0.000236003
    - **Avg. Time:** 2.36003e-06
    - **FPS:** 423723
  - **Sleep**
    - **Total Time:** 0.163267
    - **Avg. Time:** 0.00163267
    - **FPS:** 612.492
  - **GPU**
    - **Total Time:** 0.144597
    - **Avg. GPU Time:** 0.00144597
    - **FPS:** 691.578
  - **Loop**
    - **Total Time:** 0.313388
    - **Avg. Loop Time:** 0.00313388
    - **FPS:** 319.093
- As can be seen, the Write FPS is the liming factor and is therefore the upper bound on the Loop FPS. The discrepancy between the Write FPS and Loop FPS can be explained by the overhead for creating, waiting/sleeping and closing down the seperate write thread. As can be seen below, the utilization of the write thread is ~95% for 100 frames, so very high considering what has to be done. At the same time, it can be noted that the Total Thread Time = 0.313109 and the Loop Time = 0.313388, so a variance of only 0.00028 seconds between the loop and writing thread executions, highlighting that the critical path of the program is the write thread.
  - **Total Write Thread Time:** 0.313109
  - **Total Write Thread Op. Time:** 0.296184
  - **Utilization:** 0.945944
