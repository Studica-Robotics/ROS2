CXX=g++
CXXFLAGS=-I/usr/local/include/vmxpi -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread
SOURCES=$(wildcard *.cpp)
DEPS=$(SOURCES:.cpp=.d)
BINS=$(SOURCES:.cpp=)
# If using GCC version higher than 6, additionally link to libatomic.so
GCCVERSION:=$(shell gcc -dumpversion | cut -f1 -d.)
GCCVERSIONGE7:=$(shell expr $(GCCVERSION) \>= 7)
ifeq "$(GCCVERSIONGE7)" "1"
CXXFLAGS += -latomic
endif
all: $(patsubst %.cpp, %.out, $(SOURCES))

%.out:  %.cpp Makefile
        $(info GCC Version $(GCCVERSION))
        $(CXX) $< -o $(@:.out=) $(CXXFLAGS)

clean: $(patsubst %.cpp, %.clean, $(SOURCES))

%.clean:
        rm -f $(@:.clean=)

run:
        sudo ./$(BINS)

-include $(DEPS)