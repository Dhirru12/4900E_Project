# Makefile – Smart Parking Vision Producer + Parking Manager

all: qnx

CC_LINUX = gcc
CC_QNX   = qcc -Vgcc_ntox86_64

CFLAGS_COMMON = -std=c99 -Wall -Wextra -O2 -g
CFLAGS_LINUX  = $(CFLAGS_COMMON) -DVPLINUX
CFLAGS_QNX    = $(CFLAGS_COMMON)

VP_SRCS = vision_producer.c

# ── QNX ───────────────────────────────────────────────────────────────

qnx: frame_feeder_qnx parking_manager_qnx

frame_feeder_qnx: $(VP_SRCS) frame_feeder.c
	$(CC_QNX) $(CFLAGS_QNX) -DFF_STANDALONE \
	    -o $@ $^ -lsocket -lm

parking_manager_qnx: parking_manager.c vision_producer.h
	$(CC_QNX) $(CFLAGS_QNX) \
	    -o $@ parking_manager.c

# ── Clean ─────────────────────────────────────────────────────────────

clean:
	rm -f frame_feeder frame_feeder_qnx parking_manager parking_manager_qnx *.o

.PHONY: all linux qnx clean