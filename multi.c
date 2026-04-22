/* 
 * WSJT-X aggregator - collect data from two sources and feed the decodes into single wsjt-x instance.
 * Tested only with FT8.
 *
 * NOTE: When running this in standalone mode, wsjt-x and this program compete for the shared memory data
 * and things do not work quite right. Missed decodes on both sides.
 *
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/select.h>

/* Your call sign (uppercase) */
#define CALL "AA6KJ"

/* Share memory segment names (from running two separate WSJT-X's), second started with -r 2 */

#define SHM1 "WSJT-X"
#define SHM2 "WSJT-X\\ -\\ 2"

/* Note: This program replaces the standard /usr/bin/jt9 and the original becomes /usr/bin/jt9.x */
/* So: # mv /usr/bin/jt9 /usr/bin/jt9.x
 *     # cp multi /usr/bin/jt9
 */

#define JT9PATH "/usr/bin/jt9.x"  // Path to real jt9 program (jt9.x)
                                  // For standalone operation (useful for antenna testing), keep JT9PATH
                                  // as jt9 and run this program manually

/* 
 * We do not pass parameters form wsjtx to jt9. Instead we choose them manually.
 *
 * -w # fftw plan mode (0-4; default 1)
 * -m # fftw threads (default 1)
 * -M   Use multi threaded ft8 decoder
 * -C # Number of cycles for multi threaded ft8 decoder (1-3; default 3)
 * -R # Multi threaded ft8 decoder RX freq sensitivity (1-3; default 3)
 * -M # Number of threads for multi threaded ft8 decoder (0-12; default 0 (auto))
 * -E # Multi threaded ft8 decoder sensitivity (1 - 3; default 3)
 * -D # Multi threaded ft8 decoder start (0 - 4; default 3)
 * -Z   Skip multi threaded ft8 decoder DX call search
 * -d # Decoding depth (1-3; default 1)  "number of decoding passes"
 *
 */

#define JT9OPTS "-w 1 -m 3 -M -D 1 -d 3"

/*
 * Remove duplicate messages and retain the message with best report?
 * Use this with wsjt-x - not so useful in standalone mode.
 *
 */

#define REMOVE_DUPES

#define TEMP1 "/tmp/proc1"
#define TEMP2 "/tmp/proc2"

#define PROCESS_SYNC 3    // 3 seconds for letting the other jt9 process to finish

#define MAX_DECODES 256

#define MAX(a,b) ((a > b)?a:b)

pid_t proc1, proc2;
char *decodes[MAX_DECODES];

void cleanup(int x) {

  kill(proc1, SIGKILL);
  kill(proc2, SIGKILL);
  exit(0); // will also automatically free *decodes[]
}

void read_line(int fd, char *buf) {

  int i, st;

  for (i = 0; i < 512; i++) {
    if(read(fd, buf + i, 1) < 0) cleanup(0);  // Read error -> exit
    if(buf[i] == '\n') break;
  }
  buf[i+1] = '\0';
}

int get_decodes(int fd, char id, int ndecodes) {

  char *ptr;
  int i = ndecodes, gotit = 0;

  while(!gotit) {
    read_line(fd, decodes[i]);
    if(!strncmp(decodes[i], "<DecodeFinished>", 16)) gotit = 1;
    else {
      if(isdigit(*decodes[i])) {
        if((ptr = strchr(decodes[i], '~'))) *ptr = id;
        i++;
      }
    }
  }
  return i;
}

int show_decodes(int n) {

  int i, d = 0;

  for(i = 0; i < n; i++)
    if(*decodes[i] != '\0') {
      write(1, decodes[i], strlen(decodes[i]));
      d++;
    }

  return d;
}

void proc_decodes(int n) {

  int i, j, rpt1, rpt2;
  char call[64], msg1[128], msg2[128], *ptr;

  for (i = 0; i < n; i++) {
    if(*decodes[i] == '\0') {
      i++;
      continue;
    }
    sscanf(decodes[i], "%*d %*d %*f %*d %*s %*s %s", call);
    if(!strcmp(call, CALL))
      *decodes[i] = '\0'; // eliminate own call msgs on slave
    else {
      sscanf(decodes[i], "%*d %d %*f %*d %*s %[^\n]", &rpt1, msg1);
      if((ptr = strstr(msg1, "     "))) *ptr = '\0'; // remove codes
      for (j = i + 1; j < n; j++) {
	if(*decodes[j] != '\0') {     
	  sscanf(decodes[j], "%*d %d %*f %*d %*s %[^\n]", &rpt2, msg2);
	  if((ptr = strstr(msg2, "     "))) *ptr = '\0';
	  if(!strcmp(msg1, msg2)) {
	    if(rpt1 > rpt2) *decodes[j] = '\0'; // i stronger
	    else if(rpt1 < rpt2) *decodes[i] = '\0'; // j stronger
	    else {
	      *decodes[i] = '\0';
	      decodes[j][21] = '='; // indicate that a and b were equally strong
	    }
	  }
	}
      }
    }
  }
}

int main(int argc, char **argv) {

  int p1[2], p2[2], ndecodes, sync_time = -1, i;
  char buf[512];
  struct timeval tv;
  fd_set fds;

  if(!strncmp(argv[2], "WSJT-X - 2", 10)) {
    while(1) sleep(100); // do nothing - 2nd wsjt-x instance can be just minimized & ignored
                         // the second wsjt-x water fall will show the correct data
    exit(0);
  }

  pipe(p1);
  pipe(p2);

  mkdir(TEMP1, 0777);
  mkdir(TEMP2, 0777);

  if(!(proc1 = fork())) {
    char buf[128];
    close(p2[0]);
    close(p2[1]);
    close(p1[0]);
    dup2(p1[1], 1);
    chdir(TEMP1);
    sprintf(buf, "cp /usr/bin/ALLCALL7.TXT %s; %s -s %s %s", TEMP1, JT9PATH, SHM1, JT9OPTS);
    system(buf);
  }
  if(!(proc2 = fork())) {
    close(p1[0]);
    close(p1[1]);
    close(p2[0]);
    dup2(p2[1], 1);
    chdir(TEMP2);
    sprintf(buf, "cp /usr/bin/ALLCALL7.TXT %s; %s -s %s %s", TEMP2, JT9PATH, SHM2, JT9OPTS);
    system(buf);
  }
  signal(SIGINT, cleanup);

  for (i = 0; i < MAX_DECODES; i++)
    decodes[i] = (char *) malloc(sizeof(char) * 128);
  
  while(1) {
    ndecodes = 0;
    FD_ZERO(&fds);
    FD_SET(p1[0], &fds);
    FD_SET(p2[0], &fds);
    select(MAX(p1[0],p2[0])+1, &fds, NULL, NULL, NULL);
    if(FD_ISSET(p1[0], &fds)) {
      ndecodes = get_decodes(p1[0], 'a', ndecodes);
      FD_ZERO(&fds);
      FD_SET(p2[0], &fds);
      tv.tv_sec = PROCESS_SYNC;
      tv.tv_usec = 0;
      select(p2[0]+1, &fds, NULL, NULL, &tv);
      if(FD_ISSET(p2[0], &fds)) ndecodes = get_decodes(p2[0], 'b', ndecodes);
    } else if(FD_ISSET(p2[0], &fds)) {
      ndecodes = get_decodes(p2[0], 'b', ndecodes);
      FD_ZERO(&fds);
      FD_SET(p1[0], &fds);
      tv.tv_sec = PROCESS_SYNC;
      tv.tv_usec = 0;
      select(p1[0]+1, &fds, NULL, NULL, &tv);
      if(FD_ISSET(p1[0], &fds)) ndecodes = get_decodes(p1[0], 'a', ndecodes);
    }
    proc_decodes(ndecodes);
    ndecodes = show_decodes(ndecodes);
    sprintf(buf, "<DecodeFinished>   0  %d        0\n", ndecodes);
    write(1, buf, strlen(buf));
  }
}
