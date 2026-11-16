/*
 * Comparison of left (RX) vs. right (sub RX) decodes.
 *
 * Samples read at the same time, which avoids inaccuracies introduces by the
 * original multi.c program.
 *
 * Google AI generated code.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <alsa/asoundlib.h>

// Target continent: NA, AS, EU, AF, OC, AN
#define CONTINENT "EU"

// Audio & Resampling Parameters
#define CAPTURE_RATE      48000
#define DECIMATION_FACTOR 4
#define WAV_SAMPLE_RATE   (CAPTURE_RATE / DECIMATION_FACTOR) // 12,000 Hz
#define BITS_PER_SAMPLE   16
#define CAPTURE_CHANNELS  2
#define WAV_CHANNELS      1
#define CHUNK_FRAMES      1024

// Mode & Hardware Configuration
#define ALSA_DEVICE       "default" // Or "plughw:1,0" / "dsnoop:1,0"
#define CAPTURE_SECONDS   15        // 15 for FT8 ("-8"), 60 for JT9 ("-9")
#define JT9_BINARY        "/usr/bin/jt9"
#define MODE_FLAG         "-8"      // "-8" = FT8, "-9" = JT9, "-6" = JT65
#define DEPTH_FLAG        "1"       // 1 = Fast, 2 = Normal, 3 = Deep

// RAM Disk Paths
#define WAV_PATH_LEFT     "/dev/shm/jt9x_left.wav"
#define WAV_PATH_RIGHT    "/dev/shm/jt9x_right.wav"

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

static volatile sig_atomic_t keep_running = 1;

void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}

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
    int   samples;          // Number of matched decodes
    long  sum_left_db;      // Sum of Left SNR (dB)
    long  sum_right_db;     // Sum of Right SNR (dB)
    long  sum_diff_db;      // Sum of (Left - Right) dB
    int   min_diff_db;      // Minimum (Left - Right) dB seen
    int   max_diff_db;      // Maximum (Left - Right) dB seen
} StationStats;

static StationStats station_table[MAX_STATIONS];
static int station_count = 0;

typedef struct __attribute__((packed)) {
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
    if (period_sec <= 0) {
        if (utc_label && label_size > 0) utc_label[0] = '\0';
        return;
    }

    static time_t target_boundary = 0;
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    // Initialize on first call to the upcoming :00, :15, :30, or :45 boundary
    if (target_boundary == 0) {
        target_boundary = now.tv_sec + (period_sec - (now.tv_sec % period_sec));
    }

    // Only sleep if we arrived BEFORE the expected boundary
    if (now.tv_sec < target_boundary) {
        struct timespec target_ts = {
            .tv_sec = target_boundary,
            .tv_nsec = 0
        };

        // Sleep until the exact absolute wall-clock boundary
        while (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &target_ts, NULL) == EINTR) {
            continue;
        }
        clock_gettime(CLOCK_REALTIME, &now);
    }

    // Advance strictly by period_sec to lock onto :00, :15, :30, :45 without drift
    target_boundary += period_sec;

    // Optional: If we were so late that we missed multiple periods, jump to the next future boundary
    if (target_boundary <= now.tv_sec) {
        target_boundary = now.tv_sec + (period_sec - (now.tv_sec % period_sec));
    }

    struct tm gmt;
    gmtime_r(&now.tv_sec, &gmt);
    snprintf(utc_label, label_size, "%02d:%02d:%02d UTC", gmt.tm_hour, gmt.tm_min, gmt.tm_sec);
}

// Helper: check if a token looks like an amateur radio callsign (contains both letters & digits)
bool is_callsign_token(const char *tok) {
    bool has_alpha = false;
    bool has_digit = false;
    for (size_t i = 0; tok[i] != '\0'; i++) {
        if (isalpha((unsigned char)tok[i])) has_alpha = true;
        else if (isdigit((unsigned char)tok[i])) has_digit = true;
    }
    return has_alpha && has_digit && strlen(tok) >= 3;
}

// Extract the transmitting station's callsign from an FT8/JT9 payload
void extract_sender_callsign(const char *message, char *out_call, size_t out_size) {
    char buf[MAX_MSG_LEN];
    snprintf(buf, sizeof(buf), "%s", message);

    char *tokens[6] = {0};
    int n_tok = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(buf, " ", &saveptr);
    while (tok && n_tok < 6) {
        // Strip angle brackets '<' and '>' used on hashed callsigns
        if (tok[0] == '<') tok++;
        size_t len = strlen(tok);
        if (len > 0 && tok[len - 1] == '>') tok[len - 1] = '\0';

        tokens[n_tok++] = tok;
        tok = strtok_r(NULL, " ", &saveptr);
    }

    const char *chosen = "UNKNOWN";
    if (n_tok >= 2 && strcmp(tokens[0], "CQ") == 0) {
        // Handle directed CQs like "CQ DX K1ABC FN42" vs "CQ K1ABC FN42"
        if (n_tok >= 3 && !is_callsign_token(tokens[1]) && is_callsign_token(tokens[2])) {
            chosen = tokens[2];
        } else if (is_callsign_token(tokens[1])) {
            chosen = tokens[1];
        }
    } else if (n_tok >= 2 && is_callsign_token(tokens[1])) {
        // Standard exchange: "<TO_CALL> <FROM_CALL> <REPORT/GRID>"
        chosen = tokens[1];
    } else if (n_tok >= 1 && is_callsign_token(tokens[0])) {
        chosen = tokens[0];
    }

    snprintf(out_call, out_size, "%s", chosen);
}

// Record a matched station's Left vs. Right SNR into the cumulative session table
void record_station_stat(const char *callsign, int left_db, int right_db) {
    if (strcmp(callsign, "UNKNOWN") == 0) return;

    int diff = left_db - right_db;

    for (int i = 0; i < station_count; i++) {
        if (strcmp(station_table[i].callsign, callsign) == 0) {
            station_table[i].samples++;
            station_table[i].sum_left_db  += left_db;
            station_table[i].sum_right_db += right_db;
            station_table[i].sum_diff_db  += diff;
            if (diff < station_table[i].min_diff_db) station_table[i].min_diff_db = diff;
            if (diff > station_table[i].max_diff_db) station_table[i].max_diff_db = diff;
            return;
        }
    }

    if (station_count < MAX_STATIONS) {
        StationStats *st = &station_table[station_count++];
        snprintf(st->callsign, sizeof(st->callsign), "%s", callsign);
        st->samples      = 1;
        st->sum_left_db  = left_db;
        st->sum_right_db = right_db;
        st->sum_diff_db  = diff;
        st->min_diff_db  = diff;
        st->max_diff_db  = diff;
    }
}

// Comparator to sort final station table by sample count descending, then callsign ascending
int compare_station_stats(const void *a, const void *b) {
    const StationStats *sa = (const StationStats *)a;
    const StationStats *sb = (const StationStats *)b;
    if (sb->samples != sa->samples) {
        return sb->samples - sa->samples;
    }
    return strcmp(sa->callsign, sb->callsign);
}

// Print final summary table of Station Callsign and dB Difference at program exit
void print_final_station_summary(void) {
    printf("\n================================================================================\n");
    printf(" FINAL SESSION SUMMARY: STATION CALLSIGN vs. CHANNEL dB DIFFERENCE (LEFT - RIGHT)\n");
    printf("================================================================================\n");

    if (station_count == 0) {
        printf(" No common stations were decoded on both channels during this session.\n");
        printf("================================================================================\n");
        return;
    }

    qsort(station_table, station_count, sizeof(StationStats), compare_station_stats);

    printf(" %-12s | %7s | %9s | %9s | %11s | %13s\n",
           "Callsign", "Matches", "Avg Left", "Avg Right", "Avg (L - R)", "Min / Max Diff");
    printf("--------------+---------+-----------+-----------+-------------+---------------\n");

    long grand_sum_diff = 0;
    long grand_samples  = 0;

    for (int i = 0; i < station_count; i++) {
        StationStats *st = &station_table[i];
        double avg_l    = (double)st->sum_left_db  / st->samples;
        double avg_r    = (double)st->sum_right_db / st->samples;
        double avg_diff = (double)st->sum_diff_db  / st->samples;

        grand_sum_diff += st->sum_diff_db;
        grand_samples  += st->samples;

        printf(" %-12s | %7d | %+6.1f dB | %+6.1f dB | %+8.1f dB | %+4d / %+4d dB\n",
               st->callsign,
               st->samples,
               avg_l,
               avg_r,
               avg_diff,
               st->min_diff_db,
               st->max_diff_db);
    }

    printf("--------------+---------+-----------+-----------+-------------+---------------\n");
    printf(" OVERALL MEAN DIFFERENCE ACROSS ALL %ld MATCHED DECODES: %+.2f dB\n",
           grand_samples, (double)grand_sum_diff / grand_samples);
    printf("================================================================================\n");
}

bool parse_jt9_line(const char *line, DecodeEntry *entry) {
  char utc_str[16];
  char mode_sym[8];
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
  if (left_count == 0 && right_count == 0) {
    return;
  }
  printf("[%s] --- Channel dB Comparison (Left - Right) ---\n", utc_label);
  for (int i = 0; i < left_count; i++) {
#ifdef CONTINENT
    extern char *get_continent_from_callsign(char *);
    if(strcmp(get_continent_from_callsign(left_list[i].callsign), CONTINENT)) continue;
#endif
    for (int j = 0; j < right_count; j++) {
      if (!right_list[j].matched &&
	  strcmp(left_list[i].message, right_list[j].message) == 0 &&
	  abs(left_list[i].freq_hz - right_list[j].freq_hz) <= 15) {
	int diff_db = left_list[i].snr_db - right_list[j].snr_db;
	const char *stronger = (diff_db > 0) ? "LEFT stronger" :
	  (diff_db < 0) ? "RIGHT stronger" : "EQUAL";
	printf("  %-10s | %-22s | %4d Hz | L: %+3d dB | R: %+3d dB | Diff: %+3d dB (%s)\n",
	       left_list[i].callsign,
	       left_list[i].message,
	       left_list[i].freq_hz,
	       left_list[i].snr_db,
	       right_list[j].snr_db,
	       diff_db,
	       stronger);
	record_station_stat(left_list[i].callsign, left_list[i].snr_db, right_list[j].snr_db);
	left_list[i].matched = true;
	right_list[j].matched = true;
	break;
      }
    }
  }
  for (int i = 0; i < left_count; i++) {
#ifdef CONTINENT
    extern char *get_continent_from_callsign(char *);
    if(strcmp(get_continent_from_callsign(left_list[i].callsign), CONTINENT)) continue;
#endif
    if (!left_list[i].matched) {
      printf("  %-10s | %-22s | %4d Hz | L: %+3d dB | R:   ---  | LEFT only\n",
	     left_list[i].callsign, left_list[i].message, left_list[i].freq_hz, left_list[i].snr_db);
    }
  }
  for (int j = 0; j < right_count; j++) {
#ifdef CONTINENT
    extern char *get_continent_from_callsign(char *);
    if(strcmp(get_continent_from_callsign(right_list[j].callsign), CONTINENT)) continue;
#endif
    if (!right_list[j].matched) {
      printf("  %-10s | %-22s | %4d Hz | L:   ---  | R: %+3d dB | RIGHT only\n",
	     right_list[j].callsign, right_list[j].message, right_list[j].freq_hz, right_list[j].snr_db);
    }
  }
  printf("\n");
}

void run_parallel_decoders(const char *wav_left, const char *wav_right, const char *utc_label) {
  int pipe_l[2], pipe_r[2];
  if (pipe(pipe_l) < 0 || pipe(pipe_r) < 0) {
    perror("pipe");
    return;
  }
  pid_t pid_l = fork();
  if (pid_l == 0) {
    close(pipe_l[0]);
    close(pipe_r[0]);
    close(pipe_r[1]);
    dup2(pipe_l[1], STDOUT_FILENO);
    close(pipe_l[1]);
    execlp(JT9_BINARY, JT9_BINARY, MODE_FLAG, "-d", DEPTH_FLAG, wav_left, (char *)NULL);
    perror("execlp left");
    _exit(127);
  }
  pid_t pid_r = fork();
  if (pid_r == 0) {
    close(pipe_r[0]);
    close(pipe_l[0]);
    close(pipe_l[1]);
    dup2(pipe_r[1], STDOUT_FILENO);
    close(pipe_r[1]);
    execlp(JT9_BINARY, JT9_BINARY, MODE_FLAG, "-d", DEPTH_FLAG, wav_right, (char *)NULL);
    perror("execlp right");
    _exit(127);
  }
  close(pipe_l[1]);
  close(pipe_r[1]);
  DecodeEntry left_decodes[MAX_DECODES];
  DecodeEntry right_decodes[MAX_DECODES];
  int left_count = 0;
  int right_count = 0;
  FILE *fp_l = fdopen(pipe_l[0], "r");
  FILE *fp_r = fdopen(pipe_r[0], "r");
  char line[256];
  if (fp_l) {
    while (fgets(line, sizeof(line), fp_l)) {
      if (strncmp(line, "", 16) != 0) {
	printf("[%s] [LEFT]  %s", utc_label, line);
	if (left_count < MAX_DECODES && parse_jt9_line(line, &left_decodes[left_count])) {
	  left_count++;
	}
      }
    }
    fclose(fp_l);
  }
  if (fp_r) {
    while (fgets(line, sizeof(line), fp_r)) {
      if (strncmp(line, "", 16) != 0) {
	printf("[%s] [RIGHT] %s", utc_label, line);
	if (right_count < MAX_DECODES && parse_jt9_line(line, &right_decodes[right_count])) {
	  right_count++;
	}
      }
    }
    fclose(fp_r);
  }
  int status;
  if (pid_l > 0) waitpid(pid_l, &status, 0);
  if (pid_r > 0) waitpid(pid_r, &status, 0);
  print_channel_comparison(utc_label, left_decodes, left_count, right_decodes, right_count);
  fflush(stdout);
}

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

int main() {

  signal(SIGINT, handle_sigint);
  signal(SIGTERM, handle_sigint);

  snd_pcm_t *capture_handle;
  snd_pcm_hw_params_t *hw_params;
  unsigned int rate = CAPTURE_RATE;
  int err;

  if ((err = snd_pcm_open(&capture_handle, ALSA_DEVICE, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
    fprintf(stderr, "Cannot open ALSA device %s: %s\n", ALSA_DEVICE, snd_strerror(err));
    return 1;
  }
  snd_pcm_hw_params_alloca(&hw_params);
  snd_pcm_hw_params_any(capture_handle, hw_params);
  snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
  snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
  snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, 0);
  snd_pcm_hw_params_set_channels(capture_handle, hw_params, CAPTURE_CHANNELS);
  if ((err = snd_pcm_hw_params(capture_handle, hw_params)) < 0) {
    fprintf(stderr, "Cannot set ALSA parameters: %s\n", snd_strerror(err));
    snd_pcm_close(capture_handle);
    return 1;
  }

  uint32_t total_capture_frames = (CAPTURE_RATE * CAPTURE_SECONDS) - (CAPTURE_RATE / 2);
  uint32_t max_output_samples = WAV_SAMPLE_RATE * CAPTURE_SECONDS;
  int16_t *left_samples  = (int16_t *)calloc(max_output_samples, sizeof(int16_t));
  int16_t *right_samples = (int16_t *)calloc(max_output_samples, sizeof(int16_t));
  int16_t interleaved_buf[CHUNK_FRAMES * CAPTURE_CHANNELS];

  if (!left_samples || !right_samples) {
    fprintf(stderr, "Memory allocation failed.\n");
    free(left_samples);
    free(right_samples);
    snd_pcm_close(capture_handle);
    return 1;
  }
  int16_t fir_hist_l[FIR_TAPS] = {0};
  int16_t fir_hist_r[FIR_TAPS] = {0};
  char utc_label[32];
  printf("Dual-Channel RX active (%u Hz -> %d Hz, period=%ds). Press Ctrl+C to stop and print summary table.\n",
	 rate, WAV_SAMPLE_RATE, CAPTURE_SECONDS);
  while (keep_running) {
    wait_for_utc_boundary(CAPTURE_SECONDS, utc_label, sizeof(utc_label));
    if (!keep_running) break;
    snd_pcm_drop(capture_handle);
    snd_pcm_prepare(capture_handle);
    uint32_t frames_read = 0;
    uint32_t out_idx = 0;
    int decimate_phase = 0;
    while (keep_running && frames_read < total_capture_frames) {
      uint32_t frames_to_read = total_capture_frames - frames_read;
      if (frames_to_read > CHUNK_FRAMES) {
	frames_to_read = CHUNK_FRAMES;
      }
      snd_pcm_sframes_t rc = snd_pcm_readi(capture_handle, interleaved_buf, frames_to_read);
      if (rc == -EPIPE) {
	snd_pcm_prepare(capture_handle);
	continue;
      } else if (rc < 0) {
	fprintf(stderr, "ALSA read error: %s\n", snd_strerror(rc));
	break;
      }
      for (snd_pcm_sframes_t i = 0; i < rc; i++) {
	int16_t filt_l = apply_fir_step(fir_hist_l, interleaved_buf[i * 2]);
	int16_t filt_r = apply_fir_step(fir_hist_r, interleaved_buf[i * 2 + 1]);
	decimate_phase++;
	if (decimate_phase == DECIMATION_FACTOR) {
	  if (out_idx < max_output_samples) {
	    left_samples[out_idx]  = filt_l;
	    right_samples[out_idx] = filt_r;
	    out_idx++;
	  }
	  decimate_phase = 0;
	}
      }
      frames_read += rc;
    }
    if (!keep_running) break;
    memset(&left_samples[out_idx], 0, (max_output_samples - out_idx) * sizeof(int16_t));
    memset(&right_samples[out_idx], 0, (max_output_samples - out_idx) * sizeof(int16_t));
    if (write_mono_wav(WAV_PATH_LEFT, left_samples, max_output_samples) == 0 &&
	write_mono_wav(WAV_PATH_RIGHT, right_samples, max_output_samples) == 0) {
      run_parallel_decoders(WAV_PATH_LEFT, WAV_PATH_RIGHT, utc_label);
    }
  }
  // Print cumulative Station Callsign vs. dB Difference table before exiting
  print_final_station_summary();
  unlink(WAV_PATH_LEFT);
  unlink(WAV_PATH_RIGHT);
  free(left_samples);
  free(right_samples);
  snd_pcm_close(capture_handle);
  return 0;
}
