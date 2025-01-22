# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -I./bme68x -I./bmi270

# Source files
SRCS = mcu_app30_interface.c mcu_app30_support.c mcu_app30.c \
	bme68x/bme68x.c \
	bmi270/*.c\
	bmm150/*.c


# Object files
OBJS = mcu_app30_interface.o mcu_app30_support.o mcu_app30.o bme68x/bme68x.o \
	bmi270/*.o bmm150/*.o

# Executable name
TARGET = snsr

# Default target
all: $(TARGET)

# Link object files to create the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# Compile source files to object files
mcu_ap30_interface.o: mcu_ap30_interface.c
	$(CC) $(CFLAGS) -c $< -o $@

mcu_app30_support.o: mcu_app30_support.c
	$(CC) $(CFLAGS) -c $< -o $@

mcu_app30.o: mcu_app30.c
	$(CC) $(CFLAGS) -c $< -o $@

bme68x/bme68x.o: bme68x/bme68x.c
	$(CC) $(CFLAGS) -c $< -o $@

bmi270/bmi2.o: bmi270/bmi2.c
	$(CC) $(CFLAGS) -c $< -o $@

bmi270/bmi270.o: bmi270/bmi270.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean