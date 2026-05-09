multi: multi.c cty.c multi.h
	gcc -O multi.c cty.c -o multi

install:
	echo "Use install-multi script"

clean:
	rm multi *~
