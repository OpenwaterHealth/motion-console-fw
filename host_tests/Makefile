CC ?= gcc
CFLAGS ?= -Wall -Wextra -Wno-unused-parameter -O0 -g -I../Core/Inc

TESTS = test_pdc_buffer test_motion_cfg_tokens

all: $(TESTS)

test_pdc_buffer: test_pdc_buffer.c ../Core/Src/pdc_buffer.c
	$(CC) $(CFLAGS) -o $@ $<

test_motion_cfg_tokens: test_motion_cfg_tokens.c ../Core/Src/jsmn.c
	$(CC) $(CFLAGS) -o $@ $<

run: $(TESTS)
	./test_pdc_buffer
	./test_motion_cfg_tokens

clean:
	rm -f $(TESTS)
