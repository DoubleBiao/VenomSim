CC = g++


INCLUDES = -I../Eigen -I../glm-0.9.9-a2

SHARE = -march=native -O2 -fPIC -shared

SRCS = accelerometer.cpp barometer.cpp complementaryFilter.cpp esc_motor.cpp gyroscope.cpp magnetometer.cpp main.cpp PID.cpp quadcopter.cpp receiver.cpp sensorFusion.cpp stabilizer.cpp utils_diffequation.cpp camera.cpp visiondrone.cpp virtulvision.cpp polygonetool.cpp


OBJS = $(SRCS:.c=.o)

MAIN = ../build/drone_sim.so

.PHONY: depend clean

all:    $(MAIN)
	@echo drone_sim has been compiled

$(MAIN): $(OBJS) 
	$(CC) $(SHARE) $(INCLUDES) -std=c++11 -o $(MAIN) $(OBJS)

.c.o:
	$(CC) $(INCLUDES) -std=c++11 -c $<  -o $@

clean:
	$(RM) *.o *~ $(MAIN)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

