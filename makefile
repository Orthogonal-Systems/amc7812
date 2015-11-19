#==================================================================
#==================================================================
#==================================================================
# main program name
MAIN=main

#==================================================================
# compiler and linker options

SOURCES=$(MAIN).c amc7812.c serial.c
EXECUTABLE=$(MAIN)

# you shouldn't need to touch these
CC=avr-gcc
CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328p -std=c99 -c
LDFLAGS=-mmcu=atmega328p
OBJECTS=$(SOURCES:.c=.o)

#==================================================================
# hex converter options

# you shouldn't need to touch these
HEXCNVRT=avr-objcopy
HEXFLAGS=-O
HEX=$(EXECUTABLE).hex

#==================================================================
# uploader options

PARTNO=ATMEGA328P

#ifeq($(wildcard /dev/ttyACM0),) 
PORT=/dev/ttyACM*
#else
#PORT=/dev/ttyACM1
#endif

# you shouldn't need to touch these
UPLOADER=avrdude
BAUD=115200
CPROG=arduino
UPLOADFLAGS=-c $(CPROG) -b $(BAUD) 

#==================================================================
#==================================================================

all: $(SOURCES) $(EXECUTABLE) $(HEX)

#==================================================================

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

#==================================================================

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

#==================================================================

$(HEX): $(EXECUTABLE)
	$(HEXCNVRT) $(HEXFLAGS) ihex -R .eeprom $(EXECUTABLE) $@

#==================================================================

UPLOAD: $(HEX)
	$(UPLOADER) -p $(PARTNO) -P $(PORT) $(UPLOADFLAGS) -U flash:w:$(HEX)

#==================================================================

clean:
	rm *.o $(EXECUTABLE) $(HEX)

#==================================================================
#==================================================================
#==================================================================
