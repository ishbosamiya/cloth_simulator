CC = g++
INCLUDES = -I/usr/include/eigen3/

ifeq (${mode}, release)
	FLAGS = -O3 -march=native
else
	mode = debug
	FLAGS = -O3 -g
endif

GL_FLAGS = -lglfw -lGL -lX11 -lpthread -lXrandr -lXi -ldl
OBJS = glad.o main.o cloth_mesh.o constraint.o simulation.o mesh.o primitives.o
PROJECT_NAME = cloth_simulator

ifeq (${mode}, debug)
	PROJECT = ${PROJECT_NAME}_debug
else
	PROJECT = ${PROJECT_NAME}
endif

${PROJECT}: ${OBJS} clean_emacs_files
	@echo "Building on "${mode}" mode"
	@echo ".........................."
	${CC} ${INCLUDES} ${FLAGS} ${OBJS} -o $@ ${GL_FLAGS}
	-make clean

glad.o:
	${CC} -c glad.c -o $@ ${GL_FLAGS}
main.o:
	${CC} ${INCLUDES} ${FLAGS} -c main.cpp -o $@ ${GL_FLAGS}
cloth_mesh.o:
	${CC} ${INCLUDES} ${FLAGS} -c cloth_mesh.cpp -o $@ ${GL_FLAGS}
constraint.o:
	${CC} ${INCLUDES} ${FLAGS} -c constraint.cpp -o $@ ${GL_FLAGS}
simulation.o:
	${CC} ${INCLUDES} ${FLAGS} -c simulation.cpp -o $@ ${GL_FLAGS}
mesh.o:
	${CC} ${INCLUDES} ${FLAGS} -c mesh.cpp -o $@ ${GL_FLAGS}
primitives.o:
	${CC} ${INCLUDES} ${FLAGS} -c primitives.cpp -o $@ ${GL_FLAGS}

.PHONEY: clean clean_emacs_files clean_all
clean:
	-rm -rf ${OBJS}
clean_emacs_files:
	-rm -rf *~
clean_all: clean clean_emacs_files
	-rm -rf ${PROJECT_NAME} ${PROJECT_NAME}_debug
