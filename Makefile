multi: multi.c cty.c
	gcc -O multi.c cty.c -o multi

install:
	cp multi /usr/local/bin
	./local-wsjtx
	cp start-multi /usr/local/bin

clean:
	rm multi *~
