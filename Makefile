CC = g++
ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include/ -I/usr/X11/include -DOSX -Wno-deprecated-declarations
	LDFLAGS = -framework GLUT -framework OpenGL \
    	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    	-lGL -lGLU -lm -lstdc++
else
	CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iexample_00/glut-3.7.6-bin -Wno-deprecated-declarations
	LDFLAGS = -lglut -lGL -lGLU -lm
endif

RM = /bin/rm -f 
all: main 
main: example_04/example_04.o 
	$(CC) $(CFLAGS) -o as4 example_04/example_04.o $(LDFLAGS)
example_04/example_04.o: example_04/*.cpp
	$(CC) $(CFLAGS) -c example_04/*.cpp example_04/*.o
	$(CC) $(CFLAGS) -c example_04/example_04.cpp -o example_04/example_04.o
clean: 
	$(RM) *.o example_04/*.o as4


