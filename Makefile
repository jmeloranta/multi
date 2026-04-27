multi: multi.c
	gcc -g multi.c -o multi

install:
	cp multi /usr/local/bin
	./local-wsjtx
	cp start-multi /usr/local/bin

clean:
	rm multi *~
