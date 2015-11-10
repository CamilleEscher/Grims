CC = clang++
MODE = release
CMODE = -O3
ifeq ($(MODE),debug)
CMODE = -g
endif
CFLAGS = -Wall -Werror -Wextra $(CMODE) -std=c++11 -g
TARGET = grims
OBJ = main.o tools.o
LIB = -lopencv_core -lopencv_highgui -lopencv_imgproc

all : $(TARGET)

$(TARGET) : $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ) $(LIB)

main.o : main.cpp
	$(CC) $(CFLAGS) -c main.cpp

tools.o : tools.cpp tools.hpp
	$(CC) $(CFLAGS) -c tools.cpp

Staves.o : Staves.cpp Staves.hpp
	$(CC) $(CFLAGS) -c Staves.cpp

re : fclean $(TARGET)

fclean : clean
	rm -f $(TARGET)

clean :
	rm -f *.o
