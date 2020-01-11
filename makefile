CC = g++
FLAGS = -Wall
OBJS = main.o cloth_mesh.o
PROJECT = cloth_simulator

${PROJECT}: ${OBJS} clean_emacs_files
	${CC} ${FLAGS} ${OBJS} -o $@
	-make clean

debug: ${OBJS} clean_emacs_files
	${CC} ${FLAGS} -g ${OBJS} -o ${PROJECT}_debug
	-make clean

.PHONEY: clean clean_emacs_files clean_all
clean:
	-rm -rf ${OBJS}
clean_emacs_files:
	-rm -rf *~
clean_all: clean clean_emacs_files
	-rm -rf ${PROJECT} ${PROJECT}_debug
