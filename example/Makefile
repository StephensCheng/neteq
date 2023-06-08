CC = gcc
CXX = g++ -std=c++11
CFLAGS = -fno-exceptions -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g
INCPATH = -I. -I../
LIBPATH = -L. -L../neteq -L../codec/interface -L..


.PHONY: app
app:plc acce neteq

APP_PLC=AppPlc
APP_ACCE=AppAcce
APP_NETEQ=AppNeteq

plc:  $(APP_PLC)
acce: $(APP_ACCE)
neteq: $(APP_NETEQ)

AppNeteq: test_neteq.cc
	$(CXX) $< -o $@ $(INCPATH) $(LIBPATH) -lplc -lwebrtc_vad -lbase -lsignal_processing -g -lpthread -lg711
AppAcce: test_accelerate.cc
	$(CXX) $< -o $@ $(INCPATH) $(LIBPATH) -lplc -lwebrtc_vad -lbase -lsignal_processing -g -lpthread
AppPlc: test_plc.cc
	$(CXX) $< -o $@ $(INCPATH) $(LIBPATH) -lplc -lwebrtc_vad -lbase -lsignal_processing -g -lpthread

.PHONY: clean
clean:
	-rm $(APP_PLC) $(APP_ACCE) $(APP_NETEQ)
