# Compiler and flags
CXX := g++
CC := gcc
CXXFLAGS := -Wall -std=c++17
CFLAGS := -Wall
LDFLAGS :=

# Directories
SRCDIR := .
EXTRADIR := ./extras
KINEMATICSDIR := ./kinematics
CHELPERDIR := ./chelper
OBJDIR := ./obj

# Source files
CXXSRC := $(wildcard $(SRCDIR)/*.cpp) \
          $(wildcard $(EXTRADIR)/*.cpp) \
          $(wildcard $(KINEMATICSDIR)/*.cpp)
CSRC := $(wildcard $(CHELPERDIR)/*.c)

# Object files
CXXOBJ := $(patsubst %.cpp, $(OBJDIR)/%.o, $(notdir $(CXXSRC)))
COBJ := $(patsubst %.c, $(OBJDIR)/%.o, $(notdir $(CSRC)))

# Target executable
TARGET := MiniPrinter

# Default target
all: $(TARGET)

# Rule to link the target executable
$(TARGET): $(CXXOBJ) $(COBJ)
	$(CXX) $(LDFLAGS) -o $@ $^

# Rule to compile C++ source files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(EXTRADIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(KINEMATICSDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to compile C source files
$(OBJDIR)/%.o: $(CHELPERDIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create object directory if it does not exist
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Clean rule
clean:
	rm -rf $(OBJDIR) $(TARGET)

.PHONY: all clean
