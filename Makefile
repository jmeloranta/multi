multi: multi.c cty.c
	gcc -O multi.c cty.c -o multi

install:
	echo "Use install-multi script"

clean:
	rm multi *~
