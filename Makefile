CC = clang++
MODE = release
CMODE = -O3
ifeq ($(MODE),debug)
CMODE = -g -O0
endif
CFLAGS = -Wall -Werror -Wextra $(CMODE) -std=c++11
TARGET = grims
OBJ = main.o tools.o staveDetection.o Bivector.o Staves.o boundingBoxDetection.o
LIB = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

all : $(TARGET)

$(TARGET) : $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ) $(LIB)

main.o : main.cpp
	$(CC) $(CFLAGS) -c main.cpp

tools.o : tools.cpp tools.hpp
	$(CC) $(CFLAGS) -c tools.cpp

Bivector.o : Bivector.cpp Bivector.hpp
	$(CC) $(CFLAGS) -c Bivector.cpp

staveDetection.o : staveDetection.cpp staveDetection.hpp
	$(CC) $(CFLAGS) -c staveDetection.cpp

boundingBoxDetection.o : boundingBoxDetection.cpp boundingBoxDetection.hpp
	$(CC) $(CFLAGS) -c boundingBoxDetection.cpp

Staves.o : Staves.cpp Staves.hpp
	$(CC) $(CFLAGS) -c Staves.cpp

doc :
	doxygen Doxyfile

re : fclean $(TARGET)

fclean : clean
	rm -f $(TARGET)

clean :
	rm -f *.o
