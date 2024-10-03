# Compiler
CXX = g++

# Base compiler flags
BASE_CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -I./lib

# Libraries
LIBS = -lwiringPi -lpthread

# Check GCC version for filesystem library
GCC_VERSION := $(shell $(CXX) -dumpversion)
GCC_MAJOR := $(shell echo $(GCC_VERSION) | cut -f1 -d.)
GCC_MINOR := $(shell echo $(GCC_VERSION) | cut -f2 -d.)

# Add filesystem library for GCC versions before 9.1
ifeq ($(shell expr $(GCC_MAJOR) \< 9 \| \( $(GCC_MAJOR) = 9 \& $(GCC_MINOR) \< 1 \)), 1)
    LIBS += -lstdc++fs
endif

# Source files
SOURCES = bs.cpp lib/imuquat.cpp lib/pid.cpp

# Header files
HEADERS = lib/pid.h lib/rpi_hoverserial.h lib/PidConf.h

# Debug flags
DEBUG_CXXFLAGS = $(BASE_CXXFLAGS) -g -DDEBUG

# Release flags
RELEASE_CXXFLAGS = $(BASE_CXXFLAGS) -O2

# Default to release build
CXXFLAGS = $(RELEASE_CXXFLAGS)

# Executable name
EXECUTABLE = balanceSam

# Debug build settings
DEBUG_DIR = debug
DEBUG_OBJECTS = $(SOURCES:%.cpp=$(DEBUG_DIR)/%.o)
DEBUG_EXECUTABLE = $(DEBUG_DIR)/$(EXECUTABLE)

# Release build settings
RELEASE_DIR = release
RELEASE_OBJECTS = $(SOURCES:%.cpp=$(RELEASE_DIR)/%.o)
RELEASE_EXECUTABLE = $(RELEASE_DIR)/$(EXECUTABLE)

# Default target
all: release

# Debug target
debug: CXXFLAGS = $(DEBUG_CXXFLAGS)
debug: $(DEBUG_EXECUTABLE)

# Release target
release: CXXFLAGS = $(RELEASE_CXXFLAGS)
release: $(RELEASE_EXECUTABLE)

# Rule to create the debug executable
$(DEBUG_EXECUTABLE): $(DEBUG_OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Rule to create the release executable
$(RELEASE_EXECUTABLE): $(RELEASE_OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Rule to compile source files to debug object files
$(DEBUG_DIR)/%.o: %.cpp $(HEADERS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to compile source files to release object files
$(RELEASE_DIR)/%.o: %.cpp $(HEADERS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target
clean:
	rm -rf $(DEBUG_DIR) $(RELEASE_DIR)

# Phony targets
.PHONY: all debug release clean
