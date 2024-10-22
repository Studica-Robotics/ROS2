CXX = g++
CXXFLAGS = -Wall -Wextra -pedantic -I/usr/local/include/vmxpi -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread -fPIC
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)
LIBRARY = libstudica_drivers.so

# If using GCC version higher than 6, additionally link to libatomic.so
GCCVERSION := $(shell gcc -dumpversion | cut -f1 -d.)
GCCVERSIONGE7 := $(shell expr $(GCCVERSION) \>= 7)
ifeq "$(GCCVERSIONGE7)" "1"
CXXFLAGS += -latomic
endif

INCLUDE_INSTALL_PATH = /usr/local/include/studica_drivers
LIB_INSTALL_PATH = /usr/local/lib/studica_drivers/

all: $(LIBRARY)

$(LIBRARY): $(OBJS)
	$(CXX) -shared -o $@ $^ $(CXXFLAGS)

%.o: %.cpp
	$(CXX) -MMD -MP -c -o $@ $< $(CXXFLAGS)

# Install headers and library
install: $(LIBRARY)
	sudo mkdir -p $(INCLUDE_INSTALL_PATH)
	sudo cp -r $(SRCS:.cpp=.h) $(INCLUDE_INSTALL_PATH)/
	sudo mkdir -p $(LIB_INSTALL_PATH)
	sudo cp $(LIBRARY) $(LIB_INSTALL_PATH)

clean:
	rm -f $(OBJS) $(LIBRARY) $(DEPS)

-include $(DEPS)

.PHONY: all install clean
