CC = g++

INCLUDES = -I depend/Eigen -I depend/glm-0.9.9-a2 -I depend/stb-master -I include/physics -I depend/glad -I include/render

CMPORDER = -march=native -O2
SHARE = -shared

_PHYSICS = accelerometer.cpp barometer.cpp complementaryFilter.cpp esc_motor.cpp gyroscope.cpp magnetometer.cpp PID.cpp quadcopter.cpp receiver.cpp sensorFusion.cpp stabilizer.cpp utils_diffequation.cpp camera.cpp
_PHYDIR = src/physics
PHYSICS = $(patsubst %,$(_PHYDIR)/%,$(_PHYSICS))
PHYOBJ = $(PHYSICS:.cpp=.o)



RENDDEPEN = -lglfw -lGL -ldl

_RENDER = dronerender.cpp objloader.cpp landscape.cpp
_RENDDIR = src/render
RENDER = $(patsubst %,$(_RENDDIR)/%,$(_RENDER))
RENOBJ = $(RENDER:.cpp=.o)

_GLADDEN = glad.c stb_vorbis.c
GLADDEN = $(patsubst %,$(_RENDDIR)/%,$(_GLADDEN))

MAINSRC = src/main2.cpp


MAIN = build/drone_sim.so

.PHONY: depend clean

all:    $(MAIN)
	@echo drone_sim has been compiled

$(MAIN): $(PHYOBJ) $(RENOBJ)
	$(CC) $(CMPORDER) -fPIC $(SHARE)  $(INCLUDES) -std=c++11 -o $(MAIN) $(MAINSRC) $(GLADDEN) $(PHYOBJ) $(RENOBJ) $(RENDDEPEN)

$(_PHYDIR)/%.o: $(_PHYDIR)/%.cpp
	$(CC)  $(CMPORDER) $(INCLUDES) -fPIC -std=c++11   -c -o $@ $<

$(_RENDDIR)/%.o: $(_RENDDIR)/%.cpp
	$(CC) $(CMPORDER)  $(INCLUDES) -fPIC -std=c++11 -c -o $@ $<  $(RENDDEPEN)

clean:
	$(RM) *.o *~ $(MAIN)

cleanobj:
	$(RM)  $(RENOBJ) $(PHYOBJ)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

