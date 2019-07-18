SYS_PS        = n
SYS_IMX287    = n
DEBUG         = n

ifeq ($(SYS_PS),y)
CC       = gcc -m32
CXX      = g++ -m32
AR        = ar
LIBSO_USED = n
else
ifeq ($(SYS_IMX287),y)
CXX       = arm-none-linux-gnueabi-g++ 
CC        = arm-none-linux-gnueabi-gcc
AR        = arm-none-linux-gnueabi-ar
LIBSO_USED = y
endif
endif

CFLAGS    = -Wall -Wno-unused-variable -Wno-write-strings -lm -fPIC  -g 
CXXFLAGS  = 
INCLUDE   = -I ./inc \
            -I ./common \
            -I ./gilc \
            -I ./gnss \
            -I ./protocol \
            -I ./math
TARGET    = libgilcvehicle.a
TARGET_SO = libgilcvehicle.so
LIBPATH   = ../lib/

ifeq ($(DEBUG),y)
    DEBFLAGS = -O -DDEBUG
else
    DEBFLAGS = -O2 
endif
CFLAGS  += $(DEBFLAGS)

vpath %.h ./

OBJS      = gilc_out_pack.o \
			log.o \
			buffer.o \
            gnss_nmea.o \
            gnss_nmea_creat.o \
            filter.o \
            Comfunc.o \
            GILC_KF.o \
            CSINS.o \
            GILC_IOFile.o \
            GILC_Vehicle_lib.o \
           
#SRCS      = $(patsubst %.c,%.o,$(wildcard ./*.c))            
SRCS      = protocol/gilc_out_pack.cpp \
			common/log.cpp \
			common/buffer.c \
            gnss/gnss_nmea.c \
            gnss/gnss_nmea_creat.c \
            math/filter.cpp \
            gilc/Comfunc.cpp \
            gilc/GILC_KF.cpp \
            gilc/CSINS.cpp \
            gilc/GILC_IOFile.cpp \
            gilc/GILC_Vehicle_lib.cpp

$(OBJS):$(SRCS)
	$(CXX) $(CFLAGS) $(INCLUDE) -c $^

ifeq ($(LIBSO_USED),n)
all:$(OBJS)
	$(AR) rcs $(TARGET) $^
         
	mkdir -p $(LIBPATH)
	mv $(TARGET) $(LIBPATH)
else
all:$(OBJS)
	$(AR) rcs $(TARGET) $^
	$(CXX) -shared -o $(TARGET_SO) $^  
         
#打包 .o 文件到库文件 libmytest.a 
	mkdir -p $(LIBPATH)
	mv $(TARGET) $(LIBPATH)
	mv $(TARGET_SO) $(LIBPATH)
endif

clean:
	rm -f *.o
	rm -f $(LIBPATH)*
