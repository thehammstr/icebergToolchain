
### sets locations of vital objects
BINDIR = .
MBSYSTEMDIR = /usr/local/mbsystem/

# *** CHANGE THESE TO FIT YOUR LOCAL ENVIRONMENT ***
#
# Compilers
CC = gcc
#
# Compile and load flags:
CFLAGS = -O2 -DHAVE_CONFIG_H -g
LFLAGS = -L$(MBSYSTEMDIR)/lib -lmbio -lmbaux -lmbgsf -lsapi -lmr1pr -lproj -lnetcdf -lm

### make all makes the MB-system utilites and man pages
all:	$(BINDIR)/example2 \
        $(BINDIR)/example3 \
	$(BINDIR)/extractWallData

### make clean removes the object files
clean:
	rm -f *.o

### link the executables
$(BINDIR)/example2:	example2.o 
	$(CC) $(CFLAGS) example2.o  $(LFLAGS) \
		-o $(BINDIR)/example2
$(BINDIR)/example3:	example3.o 
	$(CC) $(CFLAGS) example3.o  $(LFLAGS) \
		-o $(BINDIR)/example3
$(BINDIR)/extractWallData: extractWallData.o
	$(CC) $(CFLAGS) extractWallData.o  $(LFLAGS) \
		-o $(BINDIR)/extractWallData

### compile the source files
example2.o:	example2.c
	$(CC) -c $(CFLAGS) -I$(MBSYSTEMDIR)/include example2.c
example3.o:	example3.c
	$(CC) -c $(CFLAGS) -I$(MBSYSTEMDIR)/include example3.c
extractWallData.o:	extractWallData.c
	$(CC) -c $(CFLAGS) -I$(MBSYSTEMDIR)/include extractWallData.c
