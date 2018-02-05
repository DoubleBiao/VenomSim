CC = g++


INCLUDES = -I../Eigen

SHARE = -fPIC -shared

SRCS = accelerometer.cpp barometer.cpp complementaryFilter.cpp esc_motor.cpp gyroscope.cpp magnetometer.cpp main.cpp PID.cpp quadcopter.cpp receiver.cpp sensorFusion.cpp stabilizer.cpp utils_diffequation.cpp


OBJS = $(SRCS:.c=.o)

MAIN = ../build/drone_sim.so

.PHONY: depend clean

all:    $(MAIN)
	@echo  Simple compiler named mycc has been compiled

$(MAIN): $(OBJS) 
	$(CC) $(SHARE) $(INCLUDES) -std=c++11 -o $(MAIN) $(OBJS)

.c.o:
	$(CC) $(INCLUDES) -std=c++11 -c $<  -o $@

clean:
	$(RM) *.o *~ $(MAIN)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

