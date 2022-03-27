# Significant elements of this Makefile are based on this excellent
# StackOverflow answer: https://stackoverflow.com/a/30602701

# The -MMD and -MP flags are used to generate dependency information
# about the source files. This makes it possible for make to know which
# headers each file depends on
CPPFLAGS := -MMD -MP
CXXFLAGS := -g -std=c++14 -Wall -Werror -Wextra -Wpedantic

SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

EXECUTABLE := $(BIN_DIR)/asteroids
SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
LIBRARIES := -lglut -lGLU -lGL

# Windows (cygwin)
ifeq "$(OS)" "Windows_NT"
	# Windows applications must have .exe extension
	EXECUTABLE := $(EXECUTABLE).exe
	# rm command for Windows PowerShell
	RM := del
	LIBRARIES := -lfreeglut -lglu32 -lopengl32
else
# OS X
	OS := $(shell uname)
	ifeq ($(OS), Darwin)
		LIBRARIES := -framework Carbon -framework OpenGL -framework GLUT
	endif
endif

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(LINK.cpp) -o $@ $^ $(LIBRARIES)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(COMPILE.cpp) -o $@ $<

.PHONY: all clean pristine run

run: $(EXECUTABLE)
	$(abspath $<)

clean:
	$(RM) $(wildcard $(OBJ_DIR)/*.d) $(wildcard $(OBJ_DIR)/*.o)

pristine: clean
	$(RM) $(EXECUTABLE)

-include $(OBJECTS:.o=.d)
