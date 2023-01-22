# Directory and file variables
SRCDIR		:= src
BINDIR		:= bin
BUILDDIR	:= build
INCLUDEDIR	:= include
INCLUDESUBD	:= $(shell find $(INCLUDEDIR)/* -type d)
TESTDIR		:= tests
DOCDIR		:= docs
DOXYFILE	:= $(DOCDIR)/Doxyfile

# Target variables
TARGET		:= $(BINDIR)/rc_pilot
TESTTARGET  := $(BINDIR)/test

# Browser for docs (firefox, google-chrome, etc.)
BROWSER		:= firefox

# File definitions for rules
#	Sources - *.c files including main.c for rc_pilot executable
#	Objects	- c objects created from the primary source files
#	Testsources - *.cpp files for test suite
#	Testobjects - c and cpp objects created from the test sources and primary sources excluding main.o
#	Includes - all include files, dependency of all objects
#	Docsources - Doxyfile and web files to build documents (containted in docs dir)
SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)
OBJECTS		:= $(SOURCES:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
TESTSOURCES	:= $(shell find $(TESTDIR) -type f -name *.cpp) 
TESTOBJECTS	:= $(TESTSOURCES:%.cpp=$(BUILDDIR)/%.o) $(filter-out $(BUILDDIR)/core/main.o,$(OBJECTS)) 
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')
DOCSOURCES	:= $(shell find $(DOCDIR)/src/*) $(DOXYFILE)

# Compilers, linkers and options
CC			:= gcc
CXX			:= g++
CLINKER		:= gcc
CXXLINKER	:= g++
WFLAGS		:= -Wall -Wextra # -Werror
INCLUDEFLAG	:= $(INCLUDESUBD:%=-I%)
CFLAGS		:= $(INCLUDEFLAG) 
CXXFLAGS	:= $(INCLUDEFLAG) -I/usr/local/include
OPT_FLAGS	:= -O3
LDFLAGS		:= -lm -lrt -pthread -ljson-c -lrobotcontrol

# Test linking and defines
TESTLINK 	:= -L/usr/local/lib/ -lboost_unit_test_framework $(LDFLAGS)
TESTDEF 	:= -DBOOST_TEST_DYN_LINK -DOFFBOARD_TEST

# Linking objects to build rc_pilot
$(TARGET): $(OBJECTS)
	@mkdir -p $(BINDIR)
	@$(CLINKER) -o $(@) $(OBJECTS) $(LDFLAGS)
	@echo "made: $(@)"

# Linking objects to build unit test executable
$(TESTTARGET): $(TESTOBJECTS)
	@mkdir -p $(BINDIR)
	@$(CXXLINKER) -o $(TESTTARGET) $^ $(TESTLINK)
	@echo "made: $(TESTTARGET)"

# Rule for all C objects (primary source code)
$(BUILDDIR)/%.o : $(SRCDIR)/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(OPT_FLAGS) $(DEBUGFLAG) $(WFLAGS) $< -o $(@) $(OFFBOARD_TEST)
	@echo "made: $(@)"

# Rule for test program C++ objects (test suite)
$(BUILDDIR)/tests/%.o : $(TESTDIR)/%.cpp $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CXX) -c $(CXXFLAGS) -O0 -g3 $(WFLAGS) $< -o $@ $(TESTDEF)
	@echo "made: $(@)"

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g3 -D DEBUG"
	@echo "$(TARGET) Make Debug Complete"

test: 
	$(MAKE) $(TESTTARGET) OFFBOARD_TEST="-D OFFBOARD_TEST" DEBUGFLAG="-g3"
	$(TESTTARGET)

docs:
	@cd $(DOCDIR); doxygen Doxyfile
	@$(BROWSER) docs/html/index.html > /dev/null 2>&1  &

clean:
	@rm -rvf $(BINDIR)
	@rm -rvf $(BUILDDIR)
	@touch * $(SRCDIR)/* $(INCLUDEDIR)/*
	@echo "Library Clean Complete"

.PHONY: clean docs
