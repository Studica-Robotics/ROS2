CXX = g++
CXXFLAGS = -Wall -Wextra -pedantic -I/usr/local/include/vmxpi -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread -fPIC
DRIVER_SRCS=$(wildcard *.cpp)
DRIVER_HDRS=$(DRIVER_SRCS:.cpp=.h)
# If using GCC version higher than 6, additionally link to libatomic.so
GCCVERSION:=$(shell gcc -dumpversion | cut -f1 -d.)
GCCVERSIONGE7:=$(shell expr $(GCCVERSION) \>= 7)
ifeq "$(GCCVERSIONGE7)" "1"
CXXFLAGS += -latomic
endif

# Output library
LIBRARY = libstudica_drivers.so

# Install paths
INCLUDE_INSTALL_PATH = /usr/local/include/studica_drivers
LIB_INSTALL_PATH = /usr/local/lib/studica_drivers/

# Default target
all: $(LIBRARY)

# Compile the shared library
$(LIBRARY): $(DRIVER_SRCS)
	$(CXX) -shared -o $@ $^ $(CXXFLAGS)

# Install headers and library
install: $(LIBRARY)
	sudo mkdir -p $(INCLUDE_INSTALL_PATH)
	sudo cp -r $(DRIVER_HDRS) $(INCLUDE_INSTALL_PATH)/
	sudo mkdir -p $(LIB_INSTALL_PATH)
	sudo cp $(LIBRARY) $(LIB_INSTALL_PATH)

clean:
	rm -f $(LIBRARY)

.PHONY: all install clean

# -include $(DRIVER_HDRS)