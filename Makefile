CC = gcc
CXX = g++ -std=c++11
CFLAGS = -fno-exceptions -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g
INC = -I.

.PHONY: libs
libs:dsp base vad neteq codec
# libs:dsp base vad

DSPSRC = $(wildcard common_audio/signal_processing/*.c)
DSPSRC += common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor.c
DSPOBJS = $(DSPSRC:%.c=%.o)
DSPLIB = libsignal_processing.a

BASESRC = $(wildcard rtc_base/*.cc)
BASEOBJS = $(BASESRC:%.cc=%.o)
BASELIB = libbase.a

VADSRC = $(wildcard common_audio/vad/*.c)
VADOBJS = $(VADSRC:%.c=%.o)
VADLIB = libwebrtc_vad.a

SCALESRC = common_audio/signal_processing/dot_product_with_scale.cc
SCALEOBJ = common_audio/signal_processing/dot_product_with_scale.o

dsp: $(DSPLIB)
base: $(BASELIB)
vad: $(VADLIB)

$(DSPLIB): $(DSPOBJS) $(SCALEOBJ)
	ar cr $@ $^

$(BASELIB): $(BASEOBJS)
	ar cr $@ $^

$(VADLIB): $(VADOBJS)
	ar cr $@ $^

$(DSPOBJS): %.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(INC)

$(SCALEOBJ): $(SCALESRC)
	$(CXX) $(CFLAGS) -c $< -o $@ $(INC)

$(BASEOBJS): %.o: %.cc
	$(CXX) $(CFLAGS) -c $< -o $@ $(INC)

$(VADOBJS): %.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(INC)

neteq: neteqlibs
neteqlibs:
	$(MAKE) -C ./neteq/

codec:codeclibs
codeclibs:
	$(MAKE) -C ./codec/interface/

.PHONY: clear
clear:curclear codecclear neteqclear

curclear:
	-rm $(DSPOBJS) $(DSPLIB) $(VADOBJS) $(VADLIB) $(BASEOBJS) $(BASELIB) $(DYLIBS) $(SCALEOBJ)
codecclear:
	cd ./codec/interface && $(MAKE) clean
neteqclear:
	cd ./neteq && $(MAKE) clean

.PHONY: example
example:
	cd ./example && $(MAKE)