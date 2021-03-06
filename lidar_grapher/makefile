CPP=g++

TEENSY=lidar_teensy.cpp
POINTS=point_preprocess.cpp math_util.cpp
LINES=line_find.cpp
LIST=doubly_linked_list.cpp
BOILER=boiler_find.cpp
MAIN=main.cpp
EXE=lidar_grapher

CFLAGS=--std=c++0x -g

ifeq ($(TIME),1)
	CFLAGS +=-DTIME
endif

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	INCLUDES=-I/usr/include
	LINKS=-L/usr/lib64
	ifeq ($(GUI),1)
		LINKS +=-lGL -lglfw
		CFLAGS +=-DGUI
	endif
endif
ifeq ($(UNAME_S),Darwin)
	ifeq ($(GUI),1)
		INCLUDES=-I/usr/local/Cellar/glfw/3.2.1/include 
		LINKS=-L/usr/local/Cellar/glfw/3.2.1/lib -lglfw
		CFLAGS +=-DGUI -framework OpenGl
	endif
endif


build: $(TEENSY) $(POINTS) $(LINES) $(LIST) $(BOILER) $(MAIN)
	$(CPP) -c -fPIC $(TEENSY) $(INCLUDES) $(LINKS) $(CFLAGS)
	$(CPP) -c -fPIC $(POINTS) $(INCLUDES) $(LINKS) $(CFLAGS)
	$(CPP) -c -fPIC $(LINES) $(INCLUDES) $(LINKS) $(CFLAGS)
	$(CPP) -c -fPIC $(LIST) $(INCLUDES) $(LINKS) $(CFLAGS)
	$(CPP) -c -fPIC $(BOILER) $(INCLUDES) $(LINKS) $(CFLAGS)
	$(CPP) -o $(EXE) $(MAIN) *.o $(INCLUDES) $(LINKS) $(CFLAGS)
clean:
	for file in $$(ls *.o); do rm $$file; done
	for file in $$(ls *.so); do rm $$file; done
	for file in $$(ls *.gch); do rm $$file; done
	for file in $$(ls $(EXE)); do rm $$file; done

