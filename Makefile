SYS_PS        = n
SYS_IMX287    = n
DEBUG         = n

ifeq ($(SYS_PS),y)
CC       = gcc -m32
CXX      = g++ -m32
AR        = ar
else
ifeq ($(SYS_IMX287),y)
CXX       = arm-none-linux-gnueabi-g++ 
CC        = arm-none-linux-gnueabi-gcc
AR        = arm-none-linux-gnueabi-ar
endif
endif

CFLAGS   = -Wall -Wno-unused-variable -lm  -Wno-write-strings -g 
CXXFLAGS = 
INCLUDE  = -I ./src \
            -I ./src/common \
            -I ./src/protocol \
            -I ./src/inc
		   
INCLUDE_CAL = -I ./src/inc \
            -I ./src/common \
            -I ./src/gilc \
            -I ./src/gnss \
            -I ./src/protocol \
            -I ./src/math
			
TARGET       = gilc_vehicle_main
TARGET_CAL   = gilc_vehicle_calibrated

LIBVAR   = -lgilcvehicle #链接 libgilcvehicle.a
LIBPATH  = -L./lib 

ifeq ($(DEBUG),y)
    DEBFLAGS = -O -DDEBUG 
else
    DEBFLAGS = -O2 
endif
CFLAGS  += $(DEBFLAGS)

#search paths for errorhandler.c
vpath %.cpp ./
#下行是为依赖项 apue.h 准备的，比如 [errorhandler.o:errorhandler.c apue.h] 里的 apue.h
vpath %.h ./

OBJS     = GILC_Vehicle_main.o
OBJS_CAL = GILC_Vehicle_calibrated.o
		   
SRCS     = src/GILC_Vehicle_main.cpp
SRCS_CAL = src/GILC_Vehicle_calibrated.cpp 
		   
$(OBJS):$(SRCS)
	$(CC) $(CFLAGS) $(INCLUDE) -c $^
		   
$(OBJS_CAL):$(SRCS_CAL)
	$(CXX) $(CFLAGS) $(INCLUDE_CAL) -c $^

all:$(OBJS) $(OBJS_CAL) $(LIB)
	cd ./src && make all    
#执行src/Makefile 里的 make all
	$(CXX) $(CFLAGS) $(INCLUDE) -o $(TARGET) $(OBJS) $(LIBVAR) $(LIBPATH)
	$(CXX) $(CFLAGS) $(INCLUDE_CAL) -o $(TARGET_CAL) $(OBJS_CAL) $(LIBVAR) $(LIBPATH)

clean:
	rm -f *.o
	rm -f $(TARGET) $(TARGET_CAL)
	cd ./src && make clean   
#执行src/Makefile 里的 make clean
