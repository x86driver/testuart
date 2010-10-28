TARGET = uart

all:$(TARGET)

uart:uart.c
	arm-none-linux-gnueabi-gcc -Wall -o $@ $< -g -static

clean:
	rm -rf $(TARGET)
