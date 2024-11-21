# Output file name is derived from the current directory name
OUT ?= $(notdir $(CURDIR))

# Determine the executable extension based on the OS
ifeq ($(OS), Windows_NT)
    OUT_EXT ?= .exe
else
    OUT_EXT ?=  # No extension for Linux (Ubuntu)
endif

# Compiler flags: Wall for warnings, O3 for optimization, and C++17 standard
CXXFLAGS ?= -Wall -O3 -std=c++17 -I$(SRC_DIR)  # Add -I$(SRC_DIR) for header files in SRC

# Compiler to use
CXX := g++

# Source files directory
SRC_DIR  := SRC

# Header files directory (if you have a specific directory for header files)
INC_DIR  := include

# Find all C and C++ source files in the SRC directory
SRCS_C   := $(wildcard $(SRC_DIR)/*.c)
SRCS_CPP := $(wildcard $(SRC_DIR)/*.cpp)
SRCS     := $(SRCS_C) $(SRCS_CPP)

# Object files directory
OBJ_DIR  := obj

# Map C source files to object files in the OBJ_DIR
OBJS_C   := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS_C))
# Map C++ source files to object files in the OBJ_DIR
OBJS_CPP := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS_CPP))
OBJS     := $(OBJS_C) $(OBJS_CPP)

# Default target
default: all

# Target for building the final executable
all: $(OUT)$(OUT_EXT)

# Cleaning up generated files
clean:
ifeq ($(OS), Windows_NT)
	del /Q $(OBJ_DIR)\*.o $(OUT)$(OUT_EXT) 2>nul && rmdir /Q /S $(OBJ_DIR) 2>nul
else
	rm -rf $(OBJ_DIR) $(OUT)$(OUT_EXT)
endif

# Rule to link all object files and produce the final executable
./$(OUT)$(OUT_EXT): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Rule to compile C source files into object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I$(INC_DIR) -c $< -o $@

# Rule to compile C++ source files into object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I$(INC_DIR) -c $< -o $@

# Rule to create the object directory
$(OBJ_DIR):
ifeq ($(OS), Windows_NT)
	if not exist $(OBJ_DIR) mkdir $(OBJ_DIR)
else
	mkdir -p $(OBJ_DIR)
endif

# Make sure these targets are not treated as files
.PHONY: all clean
.PRECIOUS: ./%$(OUT_EXT) $(OBJ_DIR)/%.o
