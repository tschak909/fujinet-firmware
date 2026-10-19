SHELL := /bin/bash
# Host tests: no hardware, no pico-sdk. CI runs this as a single `make` step.
CC ?= cc
CFLAGS ?= -Wall -Wextra -Werror -O1 -g -std=c11
INC = -I.. -I../include
SRC = ../src

TESTS = test_fujibus test_nesmap test_fast test_link test_busio test_fujimail

all: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

test_fujibus: test_fujibus.c $(SRC)/fujibus.c ../include/fujibus.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_fujibus.c $(SRC)/fujibus.c

test_nesmap: test_nesmap.c $(SRC)/nesmap.c ../include/nesmap.h ../include/fuji_mailbox.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_nesmap.c $(SRC)/nesmap.c

test_fast: test_fast.c $(SRC)/nesmap.c ../include/nes_fast.h ../include/nesmap.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_fast.c $(SRC)/nesmap.c

test_link: test_link.c $(SRC)/nesmap.c ../../ppu/src/ppu_link.c ../include/nes_link.h ../include/nes_fast.h ../../ppu/src/ppu_link.h
	$(CC) $(CFLAGS) $(INC) -I../../ppu/src -o $@ test_link.c $(SRC)/nesmap.c ../../ppu/src/ppu_link.c

test_busio: test_busio.c ../include/nes_cart.h ../include/nesmap.h ../include/fuji_mailbox.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_busio.c

test_fujimail: test_fujimail.c $(SRC)/fujimail.c $(SRC)/nesmap.c ../include/fujimail.h ../include/fuji_mailbox.h
	$(CC) $(CFLAGS) $(INC) -o $@ test_fujimail.c $(SRC)/fujimail.c $(SRC)/nesmap.c

clean:
	rm -f $(TESTS)

.PHONY: all clean
