CC ?= gcc
CFLAGS = -Wall -Wextra -O2
LDFLAGS = -L../lib -lgtf_dfe_opt

TARGET = gtf-dfe-tool

all: $(TARGET)

$(TARGET): gtf-dfe-tool.c ../lib/libgtf_dfe_opt.so
	$(CC) $(CFLAGS) -I../lib -I../driver -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -D -m 755 $(TARGET) /usr/local/bin/$(TARGET)

.PHONY: all clean install
