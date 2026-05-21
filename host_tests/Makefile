CC ?= gcc
CFLAGS ?= -Wall -Wextra -Wno-unused-parameter -O0 -g -I../Core/Inc

test_pdc_buffer: test_pdc_buffer.c ../Core/Src/pdc_buffer.c
	$(CC) $(CFLAGS) -o $@ $<

run: test_pdc_buffer
	./test_pdc_buffer

clean:
	rm -f test_pdc_buffer
