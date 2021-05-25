# calls:
CC         = g++
CFLAGS     = -c -O3 -std=c++17
LDFLAGS    =
EXECUTABLE = render

HEADERS		 = helpers.h scene.h geometry.h motion.h tiny_obj_loader.h
SOURCES    = render_final_project.cpp skeleton.cpp motion.cpp displaySkeleton.cpp geometry.cpp
OBJECTS    = $(SOURCES:.cpp=.o)

all: $(SOURCES) $(EXECUTABLE) $(HEADERS)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

.cpp.o: $(HEADERS)
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f *.o render
