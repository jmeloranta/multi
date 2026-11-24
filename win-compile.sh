#
# This compiles multi-cmp for windows. Compilation on Linux.
#
# If you need to specify JT9 program path:
# set JT9_PATH=C:\WSJT\wsjtx\bin\jt9.exe
# and run the program
#
x86_64-w64-mingw32-gcc multi-cmp.c continent.c -o multi-cmp.exe -static
