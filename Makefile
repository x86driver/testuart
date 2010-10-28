TARGET = uart

all:$(TARGET)

uart:uart.c
	gcc -Wall -o $@ $< -g

clean:
	rm -rf $(TARGET)
