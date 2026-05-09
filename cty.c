/*
 * CTY file processing.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PREFIX 32768
#define MAX_PREFLEN 16

char prefixes[MAX_PREFIX][MAX_PREFLEN];
int nprefixes = 0;

void read_cty(char *file, char *ct) { // ct = continent (EU, NA...) or NULL for everything

  FILE *fp;
  char buf[512], continent[32];
  int i, j, prev_j, state = 0, pass = 0;

  if(!(fp = fopen(file, "r"))) {
//    fprintf(stderr, "Can't open %s.\n", file);
    exit(1);
  }
  while (1) {
    if(fgets(buf, sizeof(buf), fp) == NULL) break;
    if(buf[0] == '#') continue;
    prev_j = 0;
    switch(state) {
    case 0:
      sscanf(buf, "%*[^:]: %*[^:]: %*[^:]: %[^:]", continent);
      state = 1;
      break;
   case 1:
      for (j = 0; ; j++) {
        if(buf[j] == ' ' || buf[j] == '=') {
          prev_j = j + 1;
          continue;
        }
        if(buf[j] == ',' || buf[j] == ';' || buf[j] == '(' || buf[j] == '[') {
          if(nprefixes == MAX_PREFIX) {
//            fprintf(stderr, "Increase MAX_PREFIX.\n");
            exit(1);
          }
          if(!ct || !strcmp(ct, continent)) {
            strncpy(prefixes[nprefixes], &buf[prev_j], j - prev_j);
            prefixes[nprefixes][j - prev_j + 1] = '\0';
//            printf("%d %s\n", nprefixes, prefixes[nprefixes]);fflush(stdout);
            nprefixes++;
          }
          if(buf[j] == '(') {
            for ( ; buf[j] != ')'; j++);
            j++;
          }
          if(buf[j] == '[') {
            for ( ; buf[j] != ']'; j++);
            j++;
          }
          prev_j = j + 1;
        }
        if(buf[j] == ';') {
          state = 0;
          break;
        }
        if(buf[j] == ',' && buf[j+1] == '\r') {
          state = 1;
          break;
        }
      }
    }
  }
//  fprintf(stderr, "Number of prefixes read = %d\n", nprefixes);
  fclose(fp);
}

int check_prefix(char *call) {  // 0 = not wanted, 1 = wanted

  int i;

  for (i = 0; i < nprefixes; i++) {
    if(!strncmp(prefixes[i], call, strlen(prefixes[i]))) return 1;
  }
  return 0;
}

