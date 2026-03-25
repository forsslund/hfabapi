CXX = g++
CXXFLAGS = -DLINUX -O2 -std=c++17
LDFLAGS = -lpthread

SRCS = hfabtest.cpp \
       ../hfab_api.cpp \
       ../hfab_serial.cpp \
       ../hfab_kinematics.cpp

TARGET = hfabtest

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: clean
