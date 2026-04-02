CC ?= gcc
CFLAGS = -Wall -Wextra -O2 -fPIC
LDFLAGS = -shared

TARGET = libgtf_dfe_opt.so
OBJS = libgtf_dfe_opt.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c libgtf_dfe_opt.h ../driver/mqnic_gtf_dfe.h
	$(CC) $(CFLAGS) -I../driver -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

install: $(TARGET)
	install -D -m 644 $(TARGET) /usr/local/lib/$(TARGET)
	install -D -m 644 libgtf_dfe_opt.h /usr/local/include/libgtf_dfe_opt.h
	ldconfig

.PHONY: all clean install
