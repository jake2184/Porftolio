
LJPEG_TURBO_INC_DIR = $(CURDIR)/bin/lib/libjpeg-turbo/include
LJPEG_TURBO_LIB_DIR = $(CURDIR)/bin/lib/libjpeg-turbo/lib

LDFLAGS = 

LDLIBS = -lm -lrt -ltbb -lOpenCL -ljpeg -lpthread

CPPFLAGS += -std=c++11 -g
CPPFLAGS += -O3
CPPFLAGS += -pthread
CPPFLAGS += -Wl,--no-as-needed
CPPFLAGS += -I include

CPPFLAGS += -I $(LJPEG_TURBO_INC_DIR)

LDFLAGS += -L $(LJPEG_TURBO_LIB_DIR)

bin/% : src/%.cpp libjpeg-turbo
	mkdir -p bin
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

libjpeg-turbo :
	chmod u+x scripts/libjpeg-turbo_setup.sh
	$(SHELL) scripts/libjpeg-turbo_setup.sh

clean-libjpeg-turbo :
	rm -rf build && rm -rf source && rm -rf packages && rm -rf bin/lib
