/* 
 * WSJT-X aggregator - collect data from two sources and feed the decodes into single wsjt-x instance (master).
 * The second wsjt-x instance (slave) is just feeding the audio data to jt9 shared memory & displays the waterfall.
 * I use this for getting decodes from two receivers in my Elecraft K4D, which have different antennas connected
 * to them. In my case RX a has inverted L on 160/80m and beverage on RX b.
 *
 * This program replaces the standard /usr/bin/jt9 and the original becomes /usr/bin/jt9.x
 * So: # mv /usr/bin/jt9 /usr/bin/jt9.x
 *     # cp multi /usr/bin
 *     # ln -s /usr/local/bin/multi /usr/local/bin/jt9
 * See local-wsjtx script
 *
 * To laumch two wsjt-x GUI processes, see start-multi script.
 * 
 * NOTES: 
 * - When running this in standalone mode, wsjt-x and this program compete for the shared memory data
 *   and things do not work quite right. Results in missed decodes on both sides.
 * - This works only with FT8.
 * - Filtering in wsjt-x improved does not work for some reason. So keep the filters disabled.
 *
 * Mode ~ is replaced by a (RX a), b (RX b), A, B depending on which RX had better SNR. The uppercase letters
 *        indicate that the difference was more than 8 dB. Also if only one RX decoded
 *        the message, it will be in uppercase.
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

/* Your call sign (uppercase) - used to exclude your own transmission from slave WSJT-X */
#define CALL "AA6KJ"

/* Share memory segment names (from running two separate WSJT-X's), second started with "wsjtx -r 2" */

#define SHM1 "WSJT-X"
#define SHM2 "WSJT-X\\ -\\ 2"

#define JT9PATH "/usr/local/bin/jt9.x"  // Path to real jt9 program (jt9.x) or put /usr/bin/jt9.x
                                  // For standalone operation (useful for antenna testing), keep JT9PATH
                                  // as jt9 and run this program manually

/* 
 * We do not pass parameters form wsjtx to jt9. Instead we choose them manually. Here are some decoding options:
 *
 * -w # fftw plan mode (0-4; default 1)
 * -m # fftw threads (default 1)
 * -M   Use multi threaded ft8 decoder
 * -C # Number of cycles for multi threaded ft8 decoder (1-3; default 3)
 * -R # Multi threaded ft8 decoder RX freq sensitivity (1-3; default 3)
 * -N # Number of threads for multi threaded ft8 decoder (0-12; default 0 (auto))
 * -E # Multi threaded ft8 decoder sensitivity (1 - 3; default 3)
 * -D # Multi threaded ft8 decoder start (0 - 4; default 3)
 * -Z   Skip multi threaded ft8 decoder DX call search
 * -d # Decoding depth (1-3; default 1)  "number of decoding passes"
 * -X # Experience based decoding (default 0). 1 = true or 0 = false.
 *
 */

#define JT9OPTS "-w 1 -m 2 -M -N 0 -D 1 -d 3 -X 1 -C 3"

// #define COLLECT_STATS "/home/eloranta/stat.out" // Collect statistics of RX signal strengths?
#define NOT_HEARD_DB (-26)   // If not heard, use this value for dB in stats

#define STAT_LOC 21       // Character location in decode where to add the source

#define EQUAL_THR  2      // Equal signal strength threshold (=)
#define UPCASE_THR 8      // Uppercase thershold for strength reporting (a vs. A and b vs. B)

#define TEMP1 "/tmp/proc1" // Each jt9 process has its own temp directory just in case
#define TEMP2 "/tmp/proc2"

#define START_WAIT   1    // Wait time before start jt9.x processes (helps avoid timing issue at the start)
#define PROCESS_SYNC 4    // 4 seconds for letting the other jt9 process to finish

#define MAX_DECODES 256

#define MAX(a,b) (((a) > (b))?(a):(b))

pid_t proc1, proc2;
#ifdef COLLECT_STATS
int stat_fd = -1;
#endif

void cleanup(int x) {

#ifdef COLLECT_STATS
  close(stat_fd);
#endif
  kill(proc1, SIGKILL);
  kill(proc2, SIGKILL);
  exit(0); // will also automatically free *decodes[]
}

void add_id(char *str, char id) {

  str[STAT_LOC] = id;
}

void read_line(int fd, char *buf) {

  int i, st;

  for (i = 0; i < 512; i++) {
    if(read(fd, buf + i, 1) < 0) cleanup(0);  // Read error -> exit
    if(buf[i] == '\n') break;
  }
  buf[i+1] = '\0';
}

#ifdef COLLECT_STATS
void collect_stats(char *msg, int rpt1, int rpt2) {

  char *err = "Error opening stat file.\n";
  char buf[128];

  if(stat_fd == -1) {
    unlink(COLLECT_STATS);
    if((stat_fd = open(COLLECT_STATS, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR)) < 0) {
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
#endif

int get_decodes(int fd, char *decodes[]) {

  int i = 0;

  while(1) {
    read_line(fd, decodes[i]);
    if(!strncmp(decodes[i], "<DecodeFinished>", 16)) break;
    i++;
  }
  return i;
}

int show_decodes(char *decodes[], int ndecodes) {

  int i, d = 0;

  for(i = 0; i < ndecodes; i++)
    if(*decodes[i] != '\0') {
      write(1, decodes[i], strlen(decodes[i]));
      d++;
    }

  return d;
}

int check_mycall(char *msg, char *mycall) {

  char p1[32], p2[32], p3[32];

  p1[0] = p2[0] = p3[0] = '\0';
  sscanf(msg, "%*d %*d %*f %*d %*s %s %s %s", p1, p2, p3);
  if(!strcmp(p2, mycall)) return 1;
  if(!strcmp(p1, "CQ") && !strcmp(p3, mycall)) return 1;
  return 0;
}

void proc_decodes(char *decodes_1[], int ndecodes1, char *decodes_2[], int ndecodes2) {

  int i, j, rpt1, rpt2, diff;
  char msg1[128], msg2[128], *ptr;

  // decodes_1 is RX a and decodes_2 RX b
  for (j = 0; j < ndecodes2; j++) { // loop over RX 2 (slave)
    if(check_mycall(decodes_2[j], CALL)) {
      *decodes_2[j] = '\0'; // eliminate own call msgs on slave
      continue;
    }
    sscanf(decodes_2[j], "%*d %d %*f %*d %*s %[^\n]", &rpt2, msg2);
    if((ptr = strstr(msg2, "     "))) *ptr = '\0'; // remove codes
    for (i = 0; i < ndecodes1; i++) { // Loop over RX 1 (master)
      if(*decodes_1[i] == '\0') continue;
      sscanf(decodes_1[i], "%*d %d %*f %*d %*s %[^\n]", &rpt1, msg1);
      if((ptr = strstr(msg1, "     "))) *ptr = '\0';
      if(!strcmp(msg1, msg2)) {
#ifdef COLLECT_STATS
        collect_stats(msg1, rpt1, rpt2);
#endif
        if(rpt1 > rpt2 && abs(rpt1-rpt2) >= EQUAL_THR) {
          *decodes_2[j] = '\0'; // i stronger
          if(abs(rpt1 - rpt2) >= UPCASE_THR) decodes_1[i][STAT_LOC] = 'A';
          else decodes_1[i][STAT_LOC] = 'a';
        } else if(rpt1 < rpt2 && abs(rpt1-rpt2) >= EQUAL_THR) {
          *decodes_1[i] = '\0'; // j stronger
          if(abs(rpt1 - rpt2) >= UPCASE_THR) decodes_2[j][STAT_LOC] = 'B';
          else decodes_2[j][STAT_LOC] = 'b';
        } else {
          *decodes_1[i] = '\0';
          add_id(decodes_2[j], '=');  // indicate that a and b were equally strong
        }
        break;
      }
    }
    if(i == ndecodes1) {
#ifdef COLLECT_STATS
      collect_stats(msg2, NOT_HEARD_DB, rpt2); // head only on slave
#endif
      decodes_2[j][STAT_LOC] = 'B'; // the other RX did not receive
      decodes_2[j][STAT_LOC+1] = '!';
    }
  }

  // Still missing received on RX 1 but not on 2.
  for (i = 0; i < ndecodes1; i++)
    if(*decodes_1[i] != '\0' && decodes_1[i][STAT_LOC] == '~') {
#ifdef COLLECT_STATS
      sscanf(decodes_1[i], "%*d %d %*f %*d %*s %[^\n]", &rpt1, msg1);
      collect_stats(msg1, rpt1, NOT_HEARD_DB); // heard only on master
#endif
      decodes_1[i][STAT_LOC] = 'A';
      decodes_1[i][STAT_LOC+1] = '!';
    }
}

int main(int argc, char **argv) {

  int p1[2], p2[2], sync_time = -1, i;
  char buf[512];
  struct timeval tv;
  fd_set fds;
  char *decodes_1[MAX_DECODES], *decodes_2[MAX_DECODES];
  int ndecodes1, ndecodes2;

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

  for (i = 0; i < MAX_DECODES; i++) {
    decodes_1[i] = (char *) malloc(sizeof(char) * 128);
    decodes_2[i] = (char *) malloc(sizeof(char) * 128);
  }

  while(1) {
    ndecodes1 = ndecodes2 = 0;
    FD_ZERO(&fds);
    FD_SET(p1[0], &fds);
    FD_SET(p2[0], &fds);
    select(MAX(p1[0],p2[0])+1, &fds, NULL, NULL, NULL);
    if(FD_ISSET(p1[0], &fds)) {
      ndecodes1 = get_decodes(p1[0], decodes_1);
      FD_ZERO(&fds);
      FD_SET(p2[0], &fds);
      tv.tv_sec = PROCESS_SYNC;
      tv.tv_usec = 0;
      select(p2[0]+1, &fds, NULL, NULL, &tv);
      if(FD_ISSET(p2[0], &fds)) ndecodes2 = get_decodes(p2[0], decodes_2);
    } else if(FD_ISSET(p2[0], &fds)) {
      ndecodes2 = get_decodes(p2[0], decodes_2);
      FD_ZERO(&fds);
      FD_SET(p1[0], &fds);
      tv.tv_sec = PROCESS_SYNC;
      tv.tv_usec = 0;
      select(p1[0]+1, &fds, NULL, NULL, &tv);
      if(FD_ISSET(p1[0], &fds)) ndecodes1 = get_decodes(p1[0], decodes_1);
    }
    proc_decodes(decodes_1, ndecodes1, decodes_2, ndecodes2);
    ndecodes1 = show_decodes(decodes_1, ndecodes1);
    ndecodes2 = show_decodes(decodes_2, ndecodes2);
    sprintf(buf, "<DecodeFinished>   0  %d        0\n", ndecodes1 + ndecodes2);
    write(1, buf, strlen(buf));
  }
}
