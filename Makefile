all: multi multi-cmp

multi: multi.c cty.c multi.h
	gcc -O multi.c cty.c -o multi

multi-cmp: multi-cmp.c continent.c
	gcc -O multi-cmp.c continent.c -o multi-cmp -lasound

install:
	echo "Use install-multi script"

clean:
	rm multi *~
