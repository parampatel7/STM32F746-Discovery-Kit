#CC = arm-none-eabi-gcc
#OBJCOPY = arm-none-eabi-objcopy
#CFLAGS = -mcpu=cortex-m7 -mthumb -Wall -g -O0 -Iinc
#LDFLAGS = -Tstm32f746.ld
#
#SRCS = main.c startup_stm32f746xx.s syscalls.c
#
#all: main.bin
#
#main.elf: $(SRCS)
#	$(CC) $(CFLAGS) $(SRCS) -o $@ $(LDFLAGS)
#
#main.bin: main.elf
#	$(OBJCOPY) -O binary $< $@
#
#clean:
#	rm -f *.o *.elf *.bin




#multiple target:dependancy possible
# led.o:led.c
#all:main.o led.o :::make command checks for dependeny

	#$(CC) $(CFLAGS) $^ -o $@
	#$^ is for dependancy,  $@ is for target
	
# Compiler
CC = arm-none-eabi-gcc

# CPU type
MACH = cortex-m7

# Flags for compilation
CFLAGS = -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0

# Linker flags (includes the linker script)
LDFLAGS = -mcpu=$(MACH) -mthumb -nostartfiles -Tstm32_ls.ld -Wl,--gc-sections

# Files
OBJS = main.o stm32_startup.o
TARGET = output.elf
BIN = output.bin

# Default target
all: $(BIN)

# Linking to create ELF
$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

# Convert ELF to BIN
$(BIN): $(TARGET)
	arm-none-eabi-objcopy -O binary $< $@

# Compile source files
main.o: main.c
	$(CC) $(CFLAGS) -o $@ $<

stm32_startup.o: stm32_startup.c
	$(CC) $(CFLAGS) -o $@ $<

# Clean generated files
clean:
	rm -f *.o *.elf *.bin
	
