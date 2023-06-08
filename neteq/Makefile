
CC = gcc
CXX = g++ -std=c++11
CFLAGS = -DNDEBUG -fno-exceptions -fvisibility=hidden -g
INC = -I. -I../


#SRC = audio_vector.cc audio_multi_vector.cc cross_correlation.cc post_decode_vad.cc\
#	background_noise.cc dsp_helper.cc random_vector.cc sync_buffer.cc expand.cc\
#	merge.cc normal.cc accelerate.cc preemptive_expand.cc time_stretch.cc

SRC = $(wildcard *.cc)

OBJS = $(SRC:%.cc=%.o)
LIB = libplc.a

plc: $(LIB)

$(LIB): $(OBJS)
	ar cr $@ $^

$(OBJS): %.o: %.cc
	$(CXX) $(CFLAGS) -c $< -o $@ $(INC)

#dylib: libplc.dylib
#
#libplc.dylib: $(OBJS)
#	$(CXX) -dynamiclib -undefined suppress -flat_namespace $^ -o $@

.PHONY: clean
clean:
	-rm $(LIB) $(OBJS)
