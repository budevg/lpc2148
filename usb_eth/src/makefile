# Sivan Toledo, 2008
# Simplified from a WinARM makefile

# makefile.local specifies the target, build type (RAM or flash), localtion of gcc, and serial port
include makefile.local

UIP_SRC = uip-1.0/uip/uip.c uip-1.0/uip/uip_arp.c uip-1.0/lib/memb.c uip-1.0/apps/telnetd/telnetd.c uip-1.0/apps/telnetd/shell.c
SRC  = $(TARGET).c $(UIP_SRC)
ASRC = startup.S

# gcc	flags
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -I. -I./uip-1.0/apps/telnetd -I./uip-1.0/uip -I./uip-1.0/lib
CFLAGS += -mcpu=arm7tdmi
#CFLAGS += -gdwarf-2
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += -Os
CFLAGS += -fno-strict-aliasing
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -std=gnu99
CFLAGS += -DLPC2148
CFLAGS += -Wall
#CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
#CFLAGS += -ffunction-sections -fdata-sections

# Assembly flags (to as via gcc)
ASFLAGS = -I. -x assembler-with-cpp
ASFLAGS += -mcpu=arm7tdmi
ASFLAGS += $(ADEFS) 
#ASFLGS += -Wa,-adhlns=$(<:.S=.lst),--gdwarf-2
#ASFLAGS += -Wa,--gdwarf-2

# linker flags
LDFLAGS = -lc -lm -lgcc -nostartfiles -T$(LDSCRIPT).ld --relocatable
LDFLAGS = -lc -lm -lgcc -nostartfiles -T$(LDSCRIPT).ld

# Define programs and commands.

# Define all object files.
COBJ      = $(SRC:.c=.o) 
AOBJ      = $(ASRC:.S=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst) 

# Compiler flags to generate dependency files.
# GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Default target.
all: build size

build: bin
bin: $(TARGET).hex

elf: $(TARGET).elf
lss: $(TARGET).lss 
sym: $(TARGET).sym

# Display size of file.
size:
	$(GCCBIN)arm-elf-size -A $(TARGET).elf

# ls -l $(TARGET).hex

# Program the device.
LPC21ISP = $(LPCBIN)lpc21isp
LPC21ISP_BAUD = 19200
LPC21ISP_BAUD = 19200
LPC21ISP_XTAL = 12000
LPC21ISP_FLASHFILE = $(TARGET).hex
# LPC21ISP_PORT set in makefile.local
# verbose output:
## LPC21ISP_DEBUG = -debug
# enter bootloader via RS232 DTR/RTS (only if hardware supports this
# feature - see Philips AppNote):
LPC21ISP_CONTROL = -control
program: $(TARGET).hex
	$(LPC21ISP) -term $(LPC21ISP_CONTROL) $(LPC21ISP_DEBUG) $(LPC21ISP_FLASHFILE) $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)
	@echo "Download done."

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
	$(GCCBIN)arm-elf-objcopy --strip-debug -O ihex $< $@

# Create extended listing file from ELF output file.
# testing: option -C
%.lss: %.elf
	$(GCCBIN)arm-elf-objdump -t -x -d -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	$(GCCBIN)arm-elf-nm -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(AOBJ) $(COBJ)
%.elf: $(AOBJ) $(COBJ) $(LDSCRIPT).ld
	$(GCCBIN)arm-elf-gcc $(CFLAGS) $(AOBJ) $(COBJ) --output $@ $(LDFLAGS)
	$(GCCBIN)arm-elf-size -A $(TARGET).elf

# Compile: create object files from C source files.
$(COBJ) : %.o : %.c
	$(GCCBIN)arm-elf-gcc -c $(CFLAGS) $(GENDEPFLAGS) $< -o $@ 

# Assemble: create object files from assembler source files.
$(AOBJ) : %.o : %.S
	$(GCCBIN)arm-elf-gcc -c $(ASFLAGS) $< -o $@

# Target: clean project.
REMOVE = rm -f
clean:
	$(REMOVE) *.hex *.elf *.map *.obj *.o *.sym *.lnk *.lss
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(COBJ)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(SRCARM:.c=.d)
	$(REMOVE) $(CPPSRC:.cpp=.s) 
	$(REMOVE) $(CPPSRC:.cpp=.d)
	$(REMOVE) $(CPPSRCARM:.cpp=.s) 
	$(REMOVE) $(CPPSRCARM:.cpp=.d)
	$(REMOVE) .dep/*

# Listing of phony targets.
#.PHONY : all begin finish end sizebefore sizeafter \
# build elf hex bin lss sym clean clean_list program

