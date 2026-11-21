/*
 * Cross-platform comparison of left (RX) vs. right (sub RX) decodes.
 * Supports Linux, macOS, and Windows.
 *
 * Make sure the paths below correspond to your system. jt9 program is part of WSJT-X or WS.
 *
 */

#ifdef _WIN32
  #define JT9_BINARY "C:\\WSJT\\wsjtx\\bin\\jt9.exe"
#elif defined(__APPLE__)
  #define JT9_BINARY "/Applications/wsjtx.app/Contents/MacOS/jt9"
#else
  #define JT9_BINARY "/usr/bin/jt9"
#endif

#ifdef _WIN32
  #define _CRT_SECURE_NO_WARNINGS
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  #include <io.h>
  #define unlink _unlink
  #define popen  _popen
  #define pclose _pclose
#else
  #define _DEFAULT_SOURCE
  #define _POSIX_C_SOURCE 200809L
  #include <unistd.h>
  #include <pthread.h>
  #include <sys/time.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio/miniaudio.h>

// Audio & Resampling Parameters
#define CAPTURE_RATE      48000
#define DECIMATION_FACTOR 4
#define WAV_SAMPLE_RATE   (CAPTURE_RATE / DECIMATION_FACTOR) // 12,000 Hz
#define BITS_PER_SAMPLE   16
#define CAPTURE_CHANNELS  2
#define WAV_CHANNELS      1

// Mode & Hardware Configuration
#define UTC_PERIOD_SEC    15        // 15 for FT8 ("-8"), 60 for JT9 ("-9")
#define CAPTURE_DEADLINE  14.0      // Stop capture 14.0s into each 15.0s UTC slot
#define MODE_FLAG         "-8"      // "-8" = FT8, "-9" = JT9, "-6" = JT65
#define DEPTH_FLAG        "1"       // 1 = Fast, 2 = Normal, 3 = Deep

// Decode & Station Tracking Limits
#define MAX_DECODES       128
#define MAX_MSG_LEN       64
#define MAX_CALLSIGN_LEN  16
#define MAX_STATIONS      1024

// 15-tap linear-phase FIR low-pass filter (cutoff ~4.5 kHz at 48 kHz)
#define FIR_TAPS 15
static const int32_t fir_coeffs[FIR_TAPS] = {
    -156, -390, -520, 0, 1638, 4194, 6684, 7700,
    6684, 4194, 1638, 0, -520, -390, -156
};
#define FIR_SCALE_SHIFT 15

// --- Cross-Platform Threading & Time Abstractions ---
#ifdef _WIN32
  typedef CRITICAL_SECTION mutex_t;
  #define mutex_init(m)    InitializeCriticalSection(m)
  #define mutex_lock(m)    EnterCriticalSection(m)
  #define mutex_unlock(m)  LeaveCriticalSection(m)
  #define sleep_ms(ms)     Sleep(ms)
  #define strtok_r         strtok_s
#else
  typedef pthread_mutex_t mutex_t;
  #define mutex_init(m)    pthread_mutex_init(m, NULL)
  #define mutex_lock(m)    pthread_mutex_lock(m)
  #define mutex_unlock(m)  pthread_mutex_unlock(m)
  static inline void sleep_ms(int ms) {
      struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };
      nanosleep(&ts, NULL);
  }
#endif

static volatile sig_atomic_t keep_running = 1;
static mutex_t stats_mutex;
static mutex_t print_mutex;
static mutex_t audio_mutex;

void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

double get_wall_time_sec(void) {
#ifdef _WIN32
    FILETIME ft;
    GetSystemTimePreciseAsFileTime(&ft);
    uint64_t ticks = ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;
    // Convert Windows epoch (1601) to Unix epoch (1970) in seconds
    return (double)(ticks - 116444736000000000ULL) / 1e7;
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
#endif
}

void get_temp_dir(char *buf, size_t size) {
#ifdef _WIN32
    GetTempPathA((DWORD)size, buf);
#elif defined(__linux__)
    snprintf(buf, size, "/dev/shm/");
#else
    snprintf(buf, size, "/tmp/");
#endif
}

// --- Ring Buffer & Audio Callback State ---
typedef struct {
    int16_t *left_buf;
    int16_t *right_buf;
    uint32_t max_samples;
    uint32_t write_idx;
    bool     capturing;
    int16_t  fir_hist_l[FIR_TAPS];
    int16_t  fir_hist_r[FIR_TAPS];
    int      decimate_phase;
} AudioCaptureState;

static AudioCaptureState cap_state;

static inline int16_t apply_fir_step(int16_t *history, int16_t new_sample) {
    memmove(&history[1], &history[0], (FIR_TAPS - 1) * sizeof(int16_t));
    history[0] = new_sample;
    int32_t acc = 0;
    for (int k = 0; k < FIR_TAPS; k++) {
        acc += (int32_t)history[k] * fir_coeffs[k];
    }
    int32_t scaled = acc >> FIR_SCALE_SHIFT;
    if (scaled > 32767) return 32767;
    if (scaled < -32768) return -32768;
    return (int16_t)scaled;
}

void data_callback(ma_device *pDevice, void *pOutput, const void *pInput, ma_uint32 frameCount) {
    (void)pDevice;
    (void)pOutput;
    if (!pInput) return;

    const int16_t *in = (const int16_t *)pInput;

    mutex_lock(&audio_mutex);
    if (cap_state.capturing) {
        for (ma_uint32 i = 0; i < frameCount; i++) {
            int16_t filt_l = apply_fir_step(cap_state.fir_hist_l, in[i * 2]);
            int16_t filt_r = apply_fir_step(cap_state.fir_hist_r, in[i * 2 + 1]);
            cap_state.decimate_phase++;
            if (cap_state.decimate_phase == DECIMATION_FACTOR) {
                if (cap_state.write_idx < cap_state.max_samples) {
                    cap_state.left_buf[cap_state.write_idx]  = filt_l;
                    cap_state.right_buf[cap_state.write_idx] = filt_r;
                    cap_state.write_idx++;
                }
                cap_state.decimate_phase = 0;
            }
        }
    }
    mutex_unlock(&audio_mutex);
}

// --- Data Structures ---
typedef struct {
    int   snr_db;
    float dt;
    int   freq_hz;
    char  callsign[MAX_CALLSIGN_LEN];
    char  message[MAX_MSG_LEN];
    bool  matched;
} DecodeEntry;

typedef struct {
    char  callsign[MAX_CALLSIGN_LEN];
    int   samples;
    int   left_only;
    int   right_only;
    long  sum_left_db;
    long  sum_right_db;
    long  sum_diff_db;
    int   min_diff_db;
    int   max_diff_db;
} StationStats;

static StationStats station_table[MAX_STATIONS];
static int station_count = 0;
char *continent = NULL;

typedef struct {
    char wav_left[256];
    char wav_right[256];
    char utc_label[64];
} DecodeJob;

#pragma pack(push, 1)
typedef struct {
    char     riff[4];
    uint32_t overall_size;
    char     wave[4];
    char     fmt_chunk_marker[4];
    uint32_t length_of_fmt;
    uint16_t format_type;
    uint16_t channels;
    uint32_t sample_rate;
    uint32_t byterate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data_chunk_header[4];
    uint32_t data_size;
} WavHeader;
#pragma pack(pop)

void populate_wav_header(WavHeader *header, uint32_t num_samples) {
    uint32_t data_bytes = num_samples * WAV_CHANNELS * (BITS_PER_SAMPLE / 8);
    memcpy(header->riff, "RIFF", 4);
    header->overall_size = data_bytes + sizeof(WavHeader) - 8;
    memcpy(header->wave, "WAVE", 4);
    memcpy(header->fmt_chunk_marker, "fmt ", 4);
    header->length_of_fmt = 16;
    header->format_type = 1;
    header->channels = WAV_CHANNELS;
    header->sample_rate = WAV_SAMPLE_RATE;
    header->byterate = WAV_SAMPLE_RATE * WAV_CHANNELS * (BITS_PER_SAMPLE / 8);
    header->block_align = WAV_CHANNELS * (BITS_PER_SAMPLE / 8);
    header->bits_per_sample = BITS_PER_SAMPLE;
    memcpy(header->data_chunk_header, "data", 4);
    header->data_size = data_bytes;
}

int write_mono_wav(const char *filepath, const int16_t *samples, uint32_t num_samples) {
    WavHeader header;
    populate_wav_header(&header, num_samples);

    FILE *fp = fopen(filepath, "wb");
    if (!fp) {
        perror("fopen wav");
        return -1;
    }
    if (fwrite(&header, sizeof(WavHeader), 1, fp) != 1 ||
        fwrite(samples, sizeof(int16_t), num_samples, fp) != num_samples) {
        perror("fwrite wav");
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

void wait_for_utc_boundary(int period_sec, char *utc_label, size_t label_size) {
    double now = get_wall_time_sec();
    time_t sec = (time_t)now;
    time_t target_sec = sec + (period_sec - (sec % period_sec));

    while (keep_running) {
        double current = get_wall_time_sec();
        double remaining = (double)target_sec - current;
        if (remaining <= 0.0) break;

        // Coarse sleep until within 15ms, then tight yield for sub-millisecond precision
        if (remaining > 0.020) {
            sleep_ms((int)((remaining - 0.015) * 1000.0));
        } else {
            sleep_ms(1);
        }
    }

    if (utc_label && label_size > 0) {
        struct tm gmt_mark;
#ifdef _WIN32
        gmtime_s(&gmt_mark, &target_sec);
#else
        gmtime_r(&target_sec, &gmt_mark);
#endif
        snprintf(utc_label, label_size, "%02d:%02d:%02d UTC",
                 gmt_mark.tm_hour, gmt_mark.tm_min, gmt_mark.tm_sec);
    }
}

bool is_callsign_token(const char *tok) {
    bool has_alpha = false, has_digit = false;
    for (size_t i = 0; tok[i] != '\0'; i++) {
        if (isalpha((unsigned char)tok[i])) has_alpha = true;
        else if (isdigit((unsigned char)tok[i])) has_digit = true;
    }
    return has_alpha && has_digit && strlen(tok) >= 3;
}

void extract_sender_callsign(const char *message, char *out_call, size_t out_size) {
    char buf[MAX_MSG_LEN];
    snprintf(buf, sizeof(buf), "%s", message);

    char *tokens[6] = {0};
    int n_tok = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(buf, " ", &saveptr);
    while (tok && n_tok < 6) {
        if (tok[0] == '<') tok++;
        size_t len = strlen(tok);
        if (len > 0 && tok[len - 1] == '>') tok[len - 1] = '\0';
        tokens[n_tok++] = tok;
        tok = strtok_r(NULL, " ", &saveptr);
    }

    const char *chosen = "UNKNOWN";
    if (n_tok >= 2 && strcmp(tokens[0], "CQ") == 0) {
        if (n_tok >= 3 && !is_callsign_token(tokens[1]) && is_callsign_token(tokens[2])) {
            chosen = tokens[2];
        } else if (is_callsign_token(tokens[1])) {
            chosen = tokens[1];
        }
    } else if (n_tok >= 2 && is_callsign_token(tokens[1])) {
        chosen = tokens[1];
    } else if (n_tok >= 1 && is_callsign_token(tokens[0])) {
        chosen = tokens[0];
    }
    snprintf(out_call, out_size, "%s", chosen);
}

static StationStats *get_or_create_station(const char *callsign) {
    for (int i = 0; i < station_count; i++) {
        if (strcmp(station_table[i].callsign, callsign) == 0) {
	  return &station_table[i];
	}
    }
    if (station_count < MAX_STATIONS) {
      StationStats *st = &station_table[station_count++];
      memset(st, 0, sizeof(StationStats));
      snprintf(st->callsign, sizeof(st->callsign), "%s", callsign);
      return st;
    }
    return NULL;
}

void record_station_stat(const char *callsign, int left_db, int right_db) {
  if (strcmp(callsign, "UNKNOWN") == 0) return;
  int diff = left_db - right_db;
  mutex_lock(&stats_mutex);
  StationStats *st = get_or_create_station(callsign);
  if (st) {
    if (st->samples == 0) {
      st->min_diff_db = diff;
      st->max_diff_db = diff;
    } else {
      if (diff < st->min_diff_db) st->min_diff_db = diff;
      if (diff > st->max_diff_db) st->max_diff_db = diff;
    }
    st->samples++;
    st->sum_left_db  += left_db;
    st->sum_right_db += right_db;
    st->sum_diff_db  += diff;
  }
  mutex_unlock(&stats_mutex);
}

void record_missed_decode(const char *callsign, bool decoded_on_left) {
  if (strcmp(callsign, "UNKNOWN") == 0) return;
  mutex_lock(&stats_mutex);
  StationStats *st = get_or_create_station(callsign);
  if (st) {
    if (decoded_on_left) st->left_only++;
    else st->right_only++;
  }
  mutex_unlock(&stats_mutex);
}

int compare_station_stats(const void *a, const void *b) {
  const StationStats *sa = (const StationStats *)a;
  const StationStats *sb = (const StationStats *)b;
  int total_a = sa->samples + sa->left_only + sa->right_only;
  int total_b = sb->samples + sb->left_only + sb->right_only;
  if (sb->samples != sa->samples) return sb->samples - sa->samples;
  if (total_b != total_a) return total_b - total_a;
  return strcmp(sa->callsign, sb->callsign);
}

void print_final_station_summary(void) {
  mutex_lock(&stats_mutex);
  mutex_lock(&print_mutex);
  printf("\n========================================================================================================\n");
  printf(" FINAL SESSION SUMMARY: STATION CALLSIGN vs. CHANNEL dB DIFFERENCE & MISSED DECODES (LEFT - RIGHT)\n");
  printf("========================================================================================================\n");
  if (station_count == 0) {
    printf(" No stations were decoded during this session.\n");
    printf("========================================================================================================\n");
    mutex_unlock(&print_mutex);
    mutex_unlock(&stats_mutex);
    return;
  }
  qsort(station_table, station_count, sizeof(StationStats), compare_station_stats);
  printf(" %-12s | %7s | %8s | %8s | %9s | %9s | %11s | %15s\n",
	 "Callsign", "Matched", "L-Only", "R-Only", "Avg Left", "Avg Right", "Avg (L - R)", "Min / Max Diff");
  printf("--------------+---------+----------+----------+-----------+-----------+-------------+-----------------\n");
  long grand_sum_diff = 0, grand_matched = 0, grand_left_only = 0, grand_right_only = 0;
  for (int i = 0; i < station_count; i++) {
    StationStats *st = &station_table[i];
    grand_matched    += st->samples;
    grand_left_only  += st->left_only;
    grand_right_only += st->right_only;
    if (st->samples > 0) {
      double avg_l    = (double)st->sum_left_db  / st->samples;
      double avg_r    = (double)st->sum_right_db / st->samples;
      double avg_diff = (double)st->sum_diff_db  / st->samples;
      grand_sum_diff += st->sum_diff_db;
      printf(" %-12s | %7d | %8d | %8d | %+6.1f dB | %+6.1f dB | %+8.1f dB | %+4d / %+4d dB\n",
	     st->callsign, st->samples, st->left_only, st->right_only,
	     avg_l, avg_r, avg_diff, st->min_diff_db, st->max_diff_db);
    } else {
      printf(" %-12s | %7d | %8d | %8d |    ---    |    ---    |     ---     |       ---\n",
	     st->callsign, st->samples, st->left_only, st->right_only);
    }
  }
  printf("--------------+---------+----------+----------+-----------+-----------+-------------+-----------------\n");
  printf(" CHANNEL TOTALS: Left Decodes = %ld | Right Decodes = %ld | Matched = %ld\n",
	 grand_matched + grand_left_only, grand_matched + grand_right_only, grand_matched);
  printf(" MISSED DECODES: Missed by Right (Left-only) = %ld | Missed by Left (Right-only) = %ld\n",
	 grand_left_only, grand_right_only);
  if (grand_matched > 0) {
    printf(" OVERALL MEAN DIFFERENCE ACROSS ALL %ld MATCHED DECODES: %+.2f dB\n",
	   grand_matched, (double)grand_sum_diff / grand_matched);
  }
  printf("========================================================================================================\n");
  mutex_unlock(&print_mutex);
  mutex_unlock(&stats_mutex);
}

bool parse_jt9_line(const char *line, DecodeEntry *entry) {
  char utc_str[16], mode_sym[8];
  int offset = 0;
  if (sscanf(line, "%15s %d %f %d %7s %n",
	     utc_str, &entry->snr_db, &entry->dt, &entry->freq_hz, mode_sym, &offset) != 5) {
    return false;
  }
  const char *msg_start = line + offset;
  while (*msg_start == ' ') msg_start++;
  snprintf(entry->message, sizeof(entry->message), "%s", msg_start);
  size_t len = strlen(entry->message);
  while (len > 0 && (entry->message[len - 1] == '\n' || entry->message[len - 1] == '\r' || entry->message[len - 1] == ' ')) {
    entry->message[--len] = '\0';
  }
  if (len == 0) return false;
  extract_sender_callsign(entry->message, entry->callsign, sizeof(entry->callsign));
  entry->matched = false;
  return true;
}

void print_channel_comparison(const char *utc_label,
			      DecodeEntry *left_list, int left_count,
			      DecodeEntry *right_list, int right_count) {
  if (left_count == 0 && right_count == 0) return;
  extern const char *get_continent_from_callsign(char *);
  printf("[%s] --- Channel dB Comparison (Left - Right) ---\n", utc_label);
  for (int i = 0; i < left_count; i++) {
    if (continent && strcmp(get_continent_from_callsign(left_list[i].callsign), continent)) continue;
    for (int j = 0; j < right_count; j++) {
      if (!right_list[j].matched &&
	  strcmp(left_list[i].message, right_list[j].message) == 0 &&
	  abs(left_list[i].freq_hz - right_list[j].freq_hz) <= 15) {
	int diff_db = left_list[i].snr_db - right_list[j].snr_db;
	const char *stronger = (diff_db > 0) ? "LEFT stronger" :
	  (diff_db < 0) ? "RIGHT stronger" : "EQUAL";
	printf("CHCMP:  %-10s | %-22s | %4d Hz | L: %+3d dB | R: %+3d dB | Diff: %+3d dB (%s)\n",
	       left_list[i].callsign, left_list[i].message, left_list[i].freq_hz,
	       left_list[i].snr_db, right_list[j].snr_db, diff_db, stronger);
	record_station_stat(left_list[i].callsign, left_list[i].snr_db, right_list[j].snr_db);
	left_list[i].matched = true;
	right_list[j].matched = true;
	break;
      }
    }
  }
  for (int i = 0; i < left_count; i++) {
    if (continent && strcmp(get_continent_from_callsign(left_list[i].callsign), continent)) continue;
    if (!left_list[i].matched) {
      printf("CHCMP:  %-10s | %-22s | %4d Hz | L: %+3d dB | R:   ---  | LEFT only\n",
	     left_list[i].callsign, left_list[i].message, left_list[i].freq_hz, left_list[i].snr_db);
      record_missed_decode(left_list[i].callsign, true);
    }
  }
  for (int j = 0; j < right_count; j++) {
    if (continent && strcmp(get_continent_from_callsign(right_list[j].callsign), continent)) continue;
    if (!right_list[j].matched) {
      printf("CHCMP:  %-10s | %-22s | %4d Hz | L:   ---  | R: %+3d dB | RIGHT only\n",
	     right_list[j].callsign, right_list[j].message, right_list[j].freq_hz, right_list[j].snr_db);
      record_missed_decode(right_list[j].callsign, false);
    }
  }
  printf("\n");
}

void run_parallel_decoders(const char *wav_left, const char *wav_right, const char *utc_label) {
  char cmd_l[512], cmd_r[512];
  snprintf(cmd_l, sizeof(cmd_l), "\"%s\" %s -d %s \"%s\"", JT9_BINARY, MODE_FLAG, DEPTH_FLAG, wav_left);
  snprintf(cmd_r, sizeof(cmd_r), "\"%s\" %s -d %s \"%s\"", JT9_BINARY, MODE_FLAG, DEPTH_FLAG, wav_right);
  // Launch both decoder processes simultaneously
  FILE *fp_l = popen(cmd_l, "r");
  FILE *fp_r = popen(cmd_r, "r");
  DecodeEntry left_decodes[MAX_DECODES];
  DecodeEntry right_decodes[MAX_DECODES];
  int left_count = 0, right_count = 0;
  char line[256];
  mutex_lock(&print_mutex);
  if (fp_l) {
    while (fgets(line, sizeof(line), fp_l)) {
      if (line[0] != '\0' && line[0] != '\n' && line[0] != '\r') {
	printf("[%s] [LEFT]  %s", utc_label, line);
	if (left_count < MAX_DECODES && parse_jt9_line(line, &left_decodes[left_count])) {
	  left_count++;
	}
      }
    }
    pclose(fp_l);
  }
  if (fp_r) {
    while (fgets(line, sizeof(line), fp_r)) {
      if (line[0] != '\0' && line[0] != '\n' && line[0] != '\r') {
	printf("[%s] [RIGHT] %s", utc_label, line);
	if (right_count < MAX_DECODES && parse_jt9_line(line, &right_decodes[right_count])) {
	  right_count++;
	}
      }
    }
    pclose(fp_r);
  }
  print_channel_comparison(utc_label, left_decodes, left_count, right_decodes, right_count);
  fflush(stdout);
  mutex_unlock(&print_mutex);
}

#ifdef _WIN32
DWORD WINAPI decoder_thread_fn(LPVOID arg) {
#else
void *decoder_thread_fn(void *arg) {
#endif
  DecodeJob *job = (DecodeJob *)arg;
  run_parallel_decoders(job->wav_left, job->wav_right, job->utc_label);
  unlink(job->wav_left);
  unlink(job->wav_right);
  free(job);
  return 0;
}
 
void spawn_decoder_thread(DecodeJob *job) {
#ifdef _WIN32
   HANDLE hThread = CreateThread(NULL, 0, decoder_thread_fn, job, 0, NULL);
   if (hThread) {
     CloseHandle(hThread);
   } else {
     decoder_thread_fn(job);
   }
#else
   pthread_t tid;
   if (pthread_create(&tid, NULL, decoder_thread_fn, job) == 0) {
     pthread_detach(tid);
   } else {
     decoder_thread_fn(job);
   }
#endif
}
 
void usage(void) {
  fprintf(stderr, "Usage: multi-cmp {ALL,NA,AS,EU,AF,OC,AN}\n");
  exit(1);
}
 
int main(int argc, char **argv) {
  if (argc != 2) usage();
  if (strcmp(argv[1], "NA") && strcmp(argv[1], "AS") && strcmp(argv[1], "EU") &&
      strcmp(argv[1], "AF") && strcmp(argv[1], "OC") && strcmp(argv[1], "AN") &&
      strcmp(argv[1], "ALL")) usage();
  continent = (!strcmp(argv[1], "ALL")) ? NULL : argv[1];
  signal(SIGINT, handle_sigint);
  signal(SIGTERM, handle_sigint);
  mutex_init(&stats_mutex);
  mutex_init(&print_mutex);
  mutex_init(&audio_mutex);
  uint32_t max_output_samples = WAV_SAMPLE_RATE * UTC_PERIOD_SEC;
  memset(&cap_state, 0, sizeof(cap_state));
  cap_state.max_samples = max_output_samples;
  cap_state.left_buf    = (int16_t *)calloc(max_output_samples, sizeof(int16_t));
  cap_state.right_buf   = (int16_t *)calloc(max_output_samples, sizeof(int16_t));
  int16_t *local_left  = (int16_t *)calloc(max_output_samples, sizeof(int16_t));
  int16_t *local_right = (int16_t *)calloc(max_output_samples, sizeof(int16_t));
  if (!cap_state.left_buf || !cap_state.right_buf || !local_left || !local_right) {
    fprintf(stderr, "Memory allocation failed.\n");
    return 1;
  }
  // Initialize cross-platform audio capture via miniaudio
  ma_device_config deviceConfig = ma_device_config_init(ma_device_type_capture);
  deviceConfig.capture.format   = ma_format_s16;
  deviceConfig.capture.channels = CAPTURE_CHANNELS;
  deviceConfig.sampleRate       = CAPTURE_RATE;
  deviceConfig.dataCallback     = data_callback;
  ma_device device;
  if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
    fprintf(stderr, "Failed to initialize audio capture device.\n");
    return 1;
  }
  if (ma_device_start(&device) != MA_SUCCESS) {
    fprintf(stderr, "Failed to start audio capture device.\n");
    ma_device_uninit(&device);
    return 1;
  }
  char temp_dir[256];
  get_temp_dir(temp_dir, sizeof(temp_dir));
  char utc_label[64];
  uint32_t cycle_id = 0;
  printf("Dual-Channel RX active (%d Hz -> %d Hz, period=%ds, wall-clock cutoff=%.1fs). Press Ctrl+C to stop.\n",
	 CAPTURE_RATE, WAV_SAMPLE_RATE, UTC_PERIOD_SEC, CAPTURE_DEADLINE);
  while (keep_running) {
    wait_for_utc_boundary(UTC_PERIOD_SEC, utc_label, sizeof(utc_label));
    if (!keep_running) break;
    double now = get_wall_time_sec();
    time_t mark_sec = (time_t)now - ((time_t)now % UTC_PERIOD_SEC);
    double stop_deadline = (double)mark_sec + CAPTURE_DEADLINE;
    // Reset capture buffer and enable recording
    mutex_lock(&audio_mutex);
    cap_state.write_idx = 0;
    cap_state.decimate_phase = 0;
    cap_state.capturing = true;
    mutex_unlock(&audio_mutex);
    // Wait until the 14.0s deadline or buffer fills
    while (keep_running) {
      if (get_wall_time_sec() >= stop_deadline) break;
      mutex_lock(&audio_mutex);
      bool full = (cap_state.write_idx >= max_output_samples);
      mutex_unlock(&audio_mutex);
      if (full) break;
      sleep_ms(10);
    }
    // Stop recording and copy samples out safely
    mutex_lock(&audio_mutex);
    cap_state.capturing = false;
    uint32_t captured = cap_state.write_idx;
    memcpy(local_left, cap_state.left_buf, captured * sizeof(int16_t));
    memcpy(local_right, cap_state.right_buf, captured * sizeof(int16_t));
    mutex_unlock(&audio_mutex);
    if (!keep_running) break;
    // Zero-pad remaining ~1.0s of the 15.0s WAV file
    if (captured < max_output_samples) {
      memset(&local_left[captured], 0, (max_output_samples - captured) * sizeof(int16_t));
      memset(&local_right[captured], 0, (max_output_samples - captured) * sizeof(int16_t));
    }
    DecodeJob *job = (DecodeJob *)malloc(sizeof(DecodeJob));
    if (job) {
      uint32_t slot = cycle_id++ % 4;
      snprintf(job->wav_left, sizeof(job->wav_left), "%sjt9x_l_%u.wav", temp_dir, slot);
      snprintf(job->wav_right, sizeof(job->wav_right), "%sjt9x_r_%u.wav", temp_dir, slot);
      snprintf(job->utc_label, sizeof(job->utc_label), "%s", utc_label);
      if (write_mono_wav(job->wav_left, local_left, max_output_samples) == 0 &&
	  write_mono_wav(job->wav_right, local_right, max_output_samples) == 0) {
	spawn_decoder_thread(job);
      } else {
	free(job);
      }
    }
  }
  sleep_ms(500);
  print_final_station_summary();
  for (uint32_t i = 0; i < 4; i++) {
    char path[256];
    snprintf(path, sizeof(path), "%sjt9x_l_%u.wav", temp_dir, i);
    unlink(path);
    snprintf(path, sizeof(path), "%sjt9x_r_%u.wav", temp_dir, i);
    unlink(path);
  }
  ma_device_uninit(&device);
  free(cap_state.left_buf);
  free(cap_state.right_buf);
  free(local_left);
  free(local_right);
  return 0;
}
