CXX = g++
CXXFLAGS = -DLINUX -O2 -std=c++17
LDFLAGS = -lpthread

SRCS = hfabtest.cpp \
       ../hfab_api.cpp \
       ../hfab_serial.cpp \
       ../hfab_kinematics.cpp

TARGET = hfabtest

all: $(TARGET) woody_viz

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

woody_viz: woody_viz.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

clean:
	rm -f $(TARGET) woody_viz

.PHONY: clean all
