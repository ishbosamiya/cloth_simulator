CC = g++
INCLUDES = -I/usr/include/eigen3/ -I/usr/include/freetype2 -I/usr/include/libpng16

ifeq (${mode}, release)
	FLAGS = -O3 -march=native
else
	mode = debug
	FLAGS = -O3 -g
	# FLAGS = -g
endif

GL_FLAGS = -lglfw -lGL -lX11 -lpthread -lXrandr -lXi -ldl
LIB_FLAGS = -lfreetype
OBJS = glad.o cloth_mesh.o constraint.o simulation.o mesh.o primitives.o bvh.o collision.o math.o gpu_immediate.o adaptive_remesh.o
PROJECT_NAME = cloth_simulator

ifeq (${mode}, debug)
	PROJECT = ${PROJECT_NAME}_debug
else
	PROJECT = ${PROJECT_NAME}
endif

${PROJECT}: ${OBJS} main.o clean_emacs_files
	@echo "Building on "${mode}" mode"
	@echo ".........................."
	${CC} ${INCLUDES} ${FLAGS} ${OBJS} main.o -o $@ ${GL_FLAGS} ${LIB_FLAGS}
	-make clean

project_test: ${OBJS} main_testing.o clean_emacs_files
	@echo "Building $@ on "${mode}" mode"
	@echo ".........................."
	${CC} ${INCLUDES} ${FLAGS} ${OBJS} main_testing.o -o $@ ${GL_FLAGS} ${LIB_FLAGS}
	-make clean

glad.o:
	${CC} -c glad.c -o $@ ${GL_FLAGS}
main.o:
	${CC} ${INCLUDES} ${FLAGS} -c main.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
main_testing.o:
	${CC} ${INCLUDES} ${FLAGS} -c main_testing.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
cloth_mesh.o:
	${CC} ${INCLUDES} ${FLAGS} -c cloth_mesh.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
constraint.o:
	${CC} ${INCLUDES} ${FLAGS} -c constraint.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
simulation.o:
	${CC} ${INCLUDES} ${FLAGS} -c simulation.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
mesh.o:
	${CC} ${INCLUDES} ${FLAGS} -c mesh.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
primitives.o:
	${CC} ${INCLUDES} ${FLAGS} -c primitives.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
bvh.o:
	${CC} ${INCLUDES} ${FLAGS} -c bvh.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
collision.o:
	${CC} ${INCLUDES} ${FLAGS} -c collision.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
math.o:
	${CC} ${INCLUDES} ${FLAGS} -c math.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
gpu_immediate.o:
	${CC} ${INCLUDES} ${FLAGS} -c gpu_immediate.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}
adaptive_remesh.o:
	${CC} ${INCLUDES} ${FLAGS} -c adaptive_remesh.cpp -o $@ ${GL_FLAGS} ${LIB_FLAGS}

.PHONEY: clean clean_emacs_files clean_all
clean:
	-rm -rf ${OBJS} main.o main_testing.o
clean_emacs_files:
	-rm -rf *~
clean_all: clean clean_emacs_files
	-rm -rf ${PROJECT_NAME} ${PROJECT_NAME}_debug project_test
