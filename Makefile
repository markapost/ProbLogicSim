PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

OBJS = ProbLogicSim.o extApi.o extApiPlatform.o shared_memory.o
LIBPATH = ${PROJECT_ROOT}ubayes
LIBPATHM = ${PROJECT_ROOT}umath

ifeq ($(BUILD_MODE),debug)
	CFLAGS += -g
else
	CFLAGS += -O2
endif

all:	ProbLogicSim

ProbLogicSim:	$(OBJS)
	$(CXX) -o $@ $^ -L$(LIBPATH) -L$(LIBPATHM) -lpthread -lrt -lubayes -lumath

%.o:	$(PROJECT_ROOT)%.cpp
	$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(CPPFLAGS) -o $@ $< -I$(LIBPATH) -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

%.o:	$(PROJECT_ROOT)%.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) -o $@ $< -I$(LIBPATH) -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

clean:
	rm -fr ProbLogicSim $(OBJS)
