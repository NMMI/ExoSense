# compiler
COMPILER = g++

# flags
CFLAGS = -c -Wall

ifeq "$(OS)"  "Windows_NT"

# folders
OBJS_FOLDER = ..\objs_win
LIB_FOLDER = ..\lib_win

else

# folders
OBJS_FOLDER = ../objs_unix
LIB_FOLDER = ../lib_unix

endif

all:$(OBJS_FOLDER)/qbmove_communications.o $(OBJS_FOLDER)/exosense_communications.o $(OBJS_FOLDER)/imuboard_communications.o $(OBJS_FOLDER) $(LIB_FOLDER)
	ar rcs $(LIB_FOLDER)/libqbmove_comm.a $(OBJS_FOLDER)/qbmove_communications.o $(OBJS_FOLDER)/exosense_communications.o $(OBJS_FOLDER)/imuboard_communications.o

$(OBJS_FOLDER)/qbmove_communications.o:qbmove_communications.cpp $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) qbmove_communications.cpp -o $(OBJS_FOLDER)/qbmove_communications.o
	
$(OBJS_FOLDER)/exosense_communications.o:exosense_communications.cpp $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) exosense_communications.cpp -o $(OBJS_FOLDER)/exosense_communications.o	

$(OBJS_FOLDER)/imuboard_communications.o:imuboard_communications.cpp $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) imuboard_communications.cpp -o $(OBJS_FOLDER)/imuboard_communications.o	
	
	
clean:
ifeq "$(OS)"  "Windows_NT"
	rmdir /Q /S $(OBJS_FOLDER)
	rmdir /Q /S $(LIB_FOLDER)
else
	rm -rf $(OBJS_FOLDER) $(LIB_FOLDER)
endif

$(OBJS_FOLDER):
	mkdir $(OBJS_FOLDER)

$(LIB_FOLDER):
	mkdir $(LIB_FOLDER)