CC = gcc
CXX = g++ -std=c++11
#CFLAGS = -fno-exceptions -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g
CFLAGS = -DNDEBUG -DWEBRTC_POSIX -g
INC = -I.

.PHONY: all
all:g711 g722

G711SRC = $(wildcard g711/*.c)
G711OBJS = $(G711SRC:%.c=%.o)
G711LIB = libg711.a
g711: $(G711LIB)
$(G711LIB): $(G711OBJS) 
	ar cr $@ $^
$(G711OBJS): %.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(INC)

G722SRC = $(wildcard g722/*.c)
G722OBJS = $(G722SRC:%.c=%.o)
G722LIB = libg722.a
g722: $(G722LIB)
$(G722LIB): $(G722OBJS) 
	ar cr $@ $^
$(G722OBJS): %.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(INC)

.PHONY: clean

clean:
	-rm $(G711OBJS) $(G711LIB) $(G722OBJS) $(G722LIB)
