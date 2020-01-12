CC = g++
FLAGS =
GL_FLAGS = -lglfw -lGL -lX11 -lpthread -lXrandr -lXi -ldl
OBJS = glad.o main.o cloth_mesh.o
PROJECT = cloth_simulator

${PROJECT}: ${OBJS} clean_emacs_files
	${CC} ${FLAGS} ${OBJS} -o $@ ${GL_FLAGS}
	-make clean

debug: ${OBJS} clean_emacs_files
	${CC} ${FLAGS} -g ${OBJS} -o ${PROJECT}_debug ${GL_FLAGS}
	-make clean

glad.o:
	${CC} -c glad.c -o $@ ${GL_FLAGS}
main.o:
	${CC} ${FLAGS} -c main.cpp -o $@ ${GL_FLAGS}
cloth_mesh.o:
	${CC} ${FLAGS} -c cloth_mesh.cpp -o $@ ${GL_FLAGS}

.PHONEY: clean clean_emacs_files clean_all
clean:
	-rm -rf ${OBJS}
clean_emacs_files:
	-rm -rf *~
clean_all: clean clean_emacs_files
	-rm -rf ${PROJECT} ${PROJECT}_debug
