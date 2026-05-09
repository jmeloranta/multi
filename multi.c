/* 
 * WSJT-X aggregator - collect data from two sources and feed the decodes into single wsjt-x instance (master).
 * The second wsjt-x instance (slave) is just feeding the audio data to jt9 shared memory & displays the waterfall.
 * I use this for getting decodes from the two receivers in my Elecraft K4D, which have different antennas connected
 * to them. In my case RX a (master) has inverted L on 160/80m and beverage on RX b (slave). Be sure to configure 
 * the audio inputs such that, for example, master WSJT-X uses the left stereo channel and the slave the right. 
 * Audio from the two RXs is transmitted from the radio that way.
 *
 * This program replaces the standard jt9 and the original becomes jt9.x. This this program acts as a middle man
 * between WSJT-X GUI and jt9 decoder.
 * 
 * Mode ~ (or +) is replaced by a (RX a), b (RX b), A, B depending on which RX had better SNR. The uppercase letters
 *        indicate that the difference was more than 8 dB. Also if only one RX decoded
 *        the message, it will be in uppercase with ! added after it.
 *
 * When performing RX antenna comparisons (COLLECT_STATS environment variable), be sure to be on band that does 
 * not have too many stations. Timing is critical here and it is possible that we miss late decodes because we need 
 * to stay within the 2 sec time tolerance.
 *
 * NOTES: 
 * - This works only with FT8.
 * - Filtering in wsjt-x improved does not work for some reason. So keep the filters disabled.
 * - I recommend keeping this separate from your standard WSJT-X program and install this under
 *   /usr/local/bin. See Makefile install section and local-wsjtx script.
 * - Make sure that your computer is sufficiently fast - otherwise it will miss decodes.
 *   Here we have to do more than twice as much work as with normal WSJT-X.
 * - We should place the relative signal info somewhere else than overwriting the mode column
 *   (~ for FT8).
 *
 * Environmental variables (set in start-multi script):
 *
 * MYCALL         My callsign (must be set)
 * COLLECT_STATS  File for collecting statistics
 * FILTER         CTY format filter file - all prefixes present in this file will be shown
 * CONTINENT      Apply continent filter to CTY file (EU, OC, AF, etc.)
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
#include <time.h>
#include "multi.h"

/* Your call sign - used to exclude your own transmission from slave WSJT-X */
char mycall[16];

/* Shared memory segment names (from running two separate WSJT-X's), second started with "wsjtx -r 2" */
#define SHM1 "WSJT-X"
#define SHM2 "WSJT-X\\ -\\ 2"
#define JT9PATH "/usr/local/bin/jt9.x"  // Path to the real jt9 program (jt9.x)

char collect_stats[256]; // File name for storing statistics
char filter_file[256];   // CTY filter file
char continent[16];      // Continent for filtering (must match CTY format)

#define NOT_HEARD_DB (-26)   // If not heard, use this value for dB in stats
#define STAT_LOC 21       // Character location in decode where to add the source
#define EQUAL_THR  2      // Equal signal strength threshold (=)
#define UPCASE_THR 8      // Uppercase thershold for strength reporting (a vs. A and b vs. B)

#define TEMP1 "/tmp/proc1" // Each jt9 process has its own temp directory just in case
#define TEMP2 "/tmp/proc2"

#define START_WAIT   1    // Wait time before start jt9.x processes (helps avoid timing issue at the start)
#define PROCESS_SYNC 2    // 2 seconds for letting the other jt9 process to finish.
                          // Making this longer is good for late decodes but then we might be late in TX
#define FT8_PERIOD   15   // FT8 period (15 s)

#define MAX_DECODES 256

// #define DEBUG "/tmp/debug.txt"         // open file for debug info

#define MAX(a,b) (((a) > (b))?(a):(b))

pid_t proc1, proc2;
int stat_fd = -1;

#ifdef DEBUG
int debug_fd = -1;
#endif

void cleanup(int x) {

  if(stat_fd != -1) close(stat_fd);
#ifdef DEBUG
  if(debug_fd != -1) close(debug_fd);
#endif
  kill(proc1, SIGKILL);
  kill(proc2, SIGKILL);
  exit(0);
}

int getgmt() {

  time_t rt;
  struct tm *gmt;
  char buf[64];

  time(&rt);
  gmt = gmtime(&rt);
  strftime(buf, sizeof(buf), "%H%M%S", gmt);
  return atoi(buf);
}

// Return 1 when decode passes filter, 0 if not
int filter(char *decode, int ign) { // when ign = 1, ignore msgs with ; in it

  char buf[32];
  
  if(*filter_file == '\0') return 1; // filter not active

  if(strchr(decode, ';')) { // show special format msgs (mshv) - TODO
    if(ign) return 0;
    return 1;
  }

  get_call(decode, buf);
  if(check_prefix(buf)) return 1;
  return 0;
}

void read_line(int fd, char *buf) {

  int i, st, l;

  for (i = 0; i < 512; i++) {
    while((l = read(fd, buf + i, 1)) != 1) if(l < 0) cleanup(0);  // Read error -> exit
    if(buf[i] == '\n') break;
  }
  buf[i+1] = '\0';
}

void collect_stats_func(char *msg, int rpt1, int rpt2) {

  char *err = "Error opening stat file.\n";
  char buf[128];

  if(stat_fd == -1) {
    unlink(collect_stats);
    if((stat_fd = open(collect_stats, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR)) < 0) {
      write(2, err, strlen(err));
      exit(0);
    }
  }
  write(stat_fd, "# ", 2);
  write(stat_fd, msg, strlen(msg));
  write(stat_fd, "\n", 1);
  sprintf(buf, "%d %d\n", rpt1, rpt2);
  write(stat_fd, buf, strlen(buf));
}

void get_decodes(int fd1, int fd2, char *decodes1[], char *decodes2[], int *nd1, int *nd2) {

  fd_set fds;
  struct timeval tv;
  int ct, dt;

  *nd1 = *nd2 = 0;
  ct = getgmt();

  while(1) {
    FD_ZERO(&fds);
    FD_SET(fd1, &fds);
    FD_SET(fd2, &fds);
    tv.tv_sec = PROCESS_SYNC;
    tv.tv_usec = 0;
    if(!select(MAX(fd1,fd2)+1, &fds, NULL, NULL, &tv)) break;
    if(FD_ISSET(fd1, &fds)) {
      while(1) {
        read_line(fd1, decodes1[*nd1]);
        if(sscanf(decodes1[*nd1], " %d", &dt) == 1 && ct - dt >= FT8_PERIOD) continue; // old late decode data, skip
        if(!strncmp(decodes1[*nd1], "<DecodeFinished>", 16)) break;
        (*nd1)++;
      }
    }
    if(FD_ISSET(fd2, &fds)) {
      while(1) {
        read_line(fd2, decodes2[*nd2]);
        if(sscanf(decodes2[*nd2], " %d", &dt) == 1 && ct - dt >= FT8_PERIOD) continue; // old late decode data, skip
        if(!strncmp(decodes2[*nd2], "<DecodeFinished>", 16)) break;
        (*nd2)++;
      }
    }
  }
}

int show_decodes(char *decodes[], int ndecodes) {

  int i, d = 0;

  for(i = 0; i < ndecodes; i++)
    if(*decodes[i] != '\0') {
      if(filter(decodes[i], 0))
        write(1, decodes[i], strlen(decodes[i]));
      d++;
    }

  return d;
}

void get_call(char *msg, char *call) {

  char p1[32], p2[32], p3[32], p4[32];
  int l;

  p1[0] = p2[0] = p3[0] = p4[0] = '\0';
  sscanf(msg, "%*d %*d %*f %*d %*s %s %s %s %s", p1, p2, p3, p4);
  /* CQ XX <call> <locator> */
  if(!strcmp(p1, "CQ") && p4[0] != '\0') {
    if(p3[0] == '<') { // Remove angular brackets
      l = strlen(p3);
      strncpy(call, p3+1, l-2);
      call[l-2] = '\0';
    } else strcpy(call, p3);
    return;
  }
  /* CQ <from_call> <locator> or <to_call> <from_call> <msg> -- does not deal with long mshv responses */
  if(p2[0] == '<') {
    l = strlen(p2);
    strncpy(call, p2+1, l-2);
    call[l-2] = '\0';
  } else strcpy(call, p2);
  return;
}

int check_call(char *msg, char *call) {

  char buf[32];

  get_call(msg, buf);
  if(!strcmp(buf, call)) return 1;
  return 0;
}

void proc_decodes(char *decodes_1[], int ndecodes1, char *decodes_2[], int ndecodes2) {

  int i, j, rpt1, rpt2, diff;
  char msg1[128], msg2[128], *ptr;

  // decodes_1 is RX a and decodes_2 RX b
  for (j = 0; j < ndecodes2; j++) { // loop over RX 2 (slave)
    if(check_call(decodes_2[j], mycall)) {
      *decodes_2[j] = '\0'; // eliminate own call msgs on slave
      continue;
    }
    sscanf(decodes_2[j], "%*d %d %*f %*d %*s %[^\n]", &rpt2, msg2);
    if((ptr = strstr(msg2, "     "))) *ptr = '\0'; // remove codes
    for (i = 0; i < ndecodes1; i++) { // Loop over RX 1 (master)
      if(*decodes_1[i] == '\0') continue;
      sscanf(decodes_1[i], "%*d %d %*f %*d %*s %[^\n]", &rpt1, msg1);
      if((ptr = strstr(msg1, "     "))) *ptr = '\0';
      if(!strcmp(msg1, msg2)) { // There is of course a chance that there is <> or other things present and this fails
        if(*collect_stats != '\0' && filter(decodes_1[i], 1))
          collect_stats_func(msg1, rpt1, rpt2);
        if(rpt1 > rpt2 && abs(rpt1-rpt2) >= EQUAL_THR) {
          *decodes_2[j] = '\0'; // i stronger, eliminate j
          if(abs(rpt1 - rpt2) >= UPCASE_THR) decodes_1[i][STAT_LOC] = 'A';
          else decodes_1[i][STAT_LOC] = 'a';
        } else if(rpt1 < rpt2 && abs(rpt1-rpt2) >= EQUAL_THR) {
          *decodes_1[i] = '\0'; // j stronger, eliminate i
          if(abs(rpt1 - rpt2) >= UPCASE_THR) decodes_2[j][STAT_LOC] = 'B';
          else decodes_2[j][STAT_LOC] = 'b';
        } else {
          *decodes_1[i] = '\0';
          decodes_2[j][STAT_LOC] = '=';  // indicate that a and b were equally strong
        }
        break;
      }
    }
    if(i == ndecodes1) {
      if(*collect_stats != '\0' && filter(decodes_2[j], 1))
        collect_stats_func(msg2, NOT_HEARD_DB, rpt2); // head only on slave
      decodes_2[j][STAT_LOC] = 'B'; // the other RX did not receive
      decodes_2[j][STAT_LOC+1] = '!';
    }
  }

  // Still missing received on RX 1 but not on 2.
  for (i = 0; i < ndecodes1; i++)
    if(*decodes_1[i] != '\0' && decodes_1[i][STAT_LOC] == '~') {
                                           // original mode present means still not processed
      if(*collect_stats != '\0' && filter(decodes_1[i], 1)) {
        sscanf(decodes_1[i], "%*d %d %*f %*d %*s %[^\n]", &rpt1, msg1);
        collect_stats_func(msg1, rpt1, NOT_HEARD_DB); // heard only on master
      }
      decodes_1[i][STAT_LOC] = 'A';
      decodes_1[i][STAT_LOC+1] = '!';
    }
}

int main(int argc, char **argv) {

  int p1[2], p2[2], sync_time = -1, i;
  char buf[512];
  char *decodes_1[MAX_DECODES], *decodes_2[MAX_DECODES], *tmp;
  int ndecodes1, ndecodes2;

  // Get parameters from environmental variables
  if(!(tmp = getenv("MYCALL"))) exit(1);
  strcpy(mycall, tmp);
  if(!(tmp = getenv("COLLECT_STATS"))) collect_stats[0] = '\0';
  else strcpy(collect_stats, tmp);
  if(!(tmp = getenv("FILTER"))) filter_file[0] = '\0';
  else strcpy(filter_file, tmp);
  if(!(tmp = getenv("CONTINENT"))) continent[0] = '\0';
  else strcpy(continent, tmp);

  if(!strncmp(argv[2], "WSJT-X - 2", 10)) {
    while(1) sleep(100); // do nothing - 2nd wsjt-x instance can be just minimized & ignored
                         // the second wsjt-x water fall will show the correct data
    exit(0);
  }

  pipe(p1);
  pipe(p2);

  mkdir(TEMP1, 0777);
  mkdir(TEMP2, 0777);

  sleep(START_WAIT);
  if(!(proc1 = fork())) {
    close(p2[0]);
    close(p2[1]);
    close(p1[0]);
    dup2(p1[1], 1);
    chdir(TEMP1);
    sprintf(buf, "cp /usr/bin/ALLCALL7.TXT %s; %s -s %s", TEMP1, JT9PATH, SHM1);
    system(buf);
  }
  if(!(proc2 = fork())) {
    close(p1[0]);
    close(p1[1]);
    close(p2[0]);
    dup2(p2[1], 1);
    chdir(TEMP2);
    sprintf(buf, "cp /usr/bin/ALLCALL7.TXT %s; %s -s %s", TEMP2, JT9PATH, SHM2);
    system(buf);
  }
  signal(SIGINT, cleanup);

  for (i = 0; i < MAX_DECODES; i++) {
    decodes_1[i] = (char *) malloc(sizeof(char) * 128);
    decodes_2[i] = (char *) malloc(sizeof(char) * 128);
  }

  if(*filter_file != '\0') read_cty(filter_file, *continent == '\0'?NULL:continent);

#ifdef DEBUG
    if((debug_fd = open(DEBUG, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR)) < 0) exit(1);
#endif

  while(1) {
    get_decodes(p1[0], p2[0], decodes_1, decodes_2, &ndecodes1, &ndecodes2);
    proc_decodes(decodes_1, ndecodes1, decodes_2, ndecodes2);
    ndecodes1 = show_decodes(decodes_1, ndecodes1);
    ndecodes2 = show_decodes(decodes_2, ndecodes2);
    sprintf(buf, "<DecodeFinished>   0  %d        0\n", ndecodes1 + ndecodes2);
    write(1, buf, strlen(buf));
  }
}
