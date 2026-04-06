all: linux

CC ?= cc
CFLAGS_COMMON = -std=c99 -Wall -Wextra -O2 -g
CFLAGS_LINUX  = $(CFLAGS_COMMON) -DVPLINUX
VP_SRCS = vision_producer.c

linux: frame_feeder_linux parking_manager_linux

frame_feeder_linux: $(VP_SRCS) frame_feeder_linux.c
	$(CC) $(CFLAGS_LINUX) -o $@ $^ -lpthread -lm

parking_manager_linux: parking_manager_linux.c vision_producer.h
	$(CC) $(CFLAGS_LINUX) -o $@ parking_manager_linux.c -lpthread

clean:
	rm -f frame_feeder_linux parking_manager_linux *.o

.PHONY: all linux clean
