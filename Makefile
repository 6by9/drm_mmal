CROSS_COMPILE ?=

CC	:= $(CROSS_COMPILE)gcc
CFLAGS	?= -O2 -W -Wall -std=gnu99 `pkg-config --cflags libdrm` -I/opt/vc/include/
LDFLAGS	?=
LIBS	:= -lrt -ldrm `pkg-config --libs libdrm` -lvcsm -lmmal -lmmal_core -lmmal_util -lmmal_vc_client -lbcm_host -lvcos -L/opt/vc/lib -pthread

%.o : %.c
	$(CC) $(CFLAGS) -g -c -o $@ $<

all: drm_mmal

drm_mmal: drm_mmal.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o
	-rm -f drm_mmal
