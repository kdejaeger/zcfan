#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define MILLIC_TO_C(n) (n / 1000)
#define FAN_CONTROL_FILE "/proc/acpi/ibm/fan"
#define TEMP_INVALID INT_MIN
#define TEMP_MIN INT_MIN + 1
#define NS_IN_SEC 1000000000L  // 1 second in nanoseconds
#define THRESHOLD_NS 200000000 // 0.2 seconds

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define DEFAULT_WATCHDOG_SECS 120
#define S_DEFAULT_WATCHDOG_SECS STR(DEFAULT_WATCHDOG_SECS)

#define info(fmt, ...) fprintf(stderr, "[INF] " fmt, ##__VA_ARGS__)
#define err(fmt, ...) fprintf(stderr, "[ERR] " fmt, ##__VA_ARGS__)
#define expect(x)                                                              \
    do {                                                                       \
        if (!(x)) {                                                            \
            fprintf(stderr, "FATAL: !(%s) at %s:%s:%d\n", #x, __FILE__,        \
                    __func__, __LINE__);                                       \
            abort();                                                           \
        }                                                                      \
    } while (0)

#define CONFIG_MAX_STRLEN 15
#define S_CONFIG_MAX_STRLEN STR(CONFIG_MAX_STRLEN)

#define MAX_IGNORED_SENSORS 1024
#define SENSOR_NAME_MAX 256
static char ignored_sensors_arr[MAX_IGNORED_SENSORS][SENSOR_NAME_MAX];
static size_t num_to_ignore_sensors = 0;

#define MAX_SENSOR_FDS 4096
/* Relative to /sys/class/hwmon: "hwmon4294967295/temp2147483647_input" fits */
#define SENSOR_PATH_MAX 48
enum SensorKind {
    SENSOR_OTHER,
    /* The ACPI/EC CPU-level reading: what the platform's critical-shutdown
     * trip reacts to; tracked for the fan-control maximum. */
    SENSOR_CPU,
    /* Per-core die readings: averaged for the fan-control baseline. */
    SENSOR_CPU_CORE,
};
struct Sensor {
    int fd;
    enum SensorKind kind;
    char path[SENSOR_PATH_MAX];
    /* Identity (from fstat) of the opened attribute file: catches a driver
     * re-registering at the same hwmonN path, where path and kind alone
     * would not reveal that the old fds turned stale. */
    dev_t dev;
    ino_t ino;
};
struct SensorSet {
    struct Sensor sensors[MAX_SENSOR_FDS];
    size_t num_sensor_fds;
    size_t num_cpu_temp_sensors;
    size_t num_cpu_core_sensors;
    size_t num_ignored_sensors;
    /* False when the scan could not be completed (failed open, readdir
     * error, overflow); such a snapshot must not replace the active set. A
     * persistently failing sensor therefore pins the active set: its stale
     * fd either still reads or is excluded as invalid, so this is safe. */
    bool complete;
};
static struct SensorSet sensor_set;
static struct SensorSet sensor_set_scratch;
/* Overridable so tests can run against a synthetic hwmon tree. */
static const char *hwmon_root = "/sys/class/hwmon";

/* Must be highest to lowest temp */
enum FanLevel { FAN_MAX, FAN_MED, FAN_LOW, FAN_OFF, FAN_INVALID };
struct Rule {
    char tpacpi_level[CONFIG_MAX_STRLEN + 1];
    int threshold;
    const char *name;
    int debounce_secs;
};
static struct Rule rules[] = {
    [FAN_MAX] = {"full-speed", 90, "maximum", 10},
    [FAN_MED] = {"4", 80, "medium", 30},
    [FAN_LOW] = {"1", 70, "low", 60},
    [FAN_OFF] = {"0", TEMP_MIN, "off", 0},
};

static struct timespec last_watchdog_ping = {0, 0};
static time_t watchdog_secs = DEFAULT_WATCHDOG_SECS;
static int temp_hysteresis = 20;
static const unsigned int tick_hysteresis = 3;
static char output_buf[512];
static const struct Rule *current_rule = NULL;
static unsigned int level_ticks[FAN_INVALID];
static volatile sig_atomic_t run = 1;
static volatile sig_atomic_t pending_sleep = 0;
static volatile sig_atomic_t pending_resume = 0;
static int first_tick = 1; /* Stop running if errors are immediate */

enum resume_state {
    RESUME_NOT_DETECTED,
    RESUME_DETECTED,
};

static void exit_if_first_tick(void) {
    if (first_tick) {
        err("Quitting due to failure during first run\n");
        exit(1);
    }
}

static int64_t timespec_diff_ns(const struct timespec *start,
                                const struct timespec *end) {
    return ((int64_t)end->tv_sec - (int64_t)start->tv_sec) * NS_IN_SEC +
           (end->tv_nsec - start->tv_nsec);
}

static enum resume_state detect_suspend(void) {
    static struct timespec monotonic_prev, boottime_prev;
    struct timespec monotonic_now, boottime_now;

    expect(clock_gettime(CLOCK_MONOTONIC_COARSE, &monotonic_now) == 0);
    expect(clock_gettime(CLOCK_BOOTTIME, &boottime_now) == 0);

    if (monotonic_prev.tv_sec == 0 && monotonic_prev.tv_nsec == 0) {
        monotonic_prev = monotonic_now;
        boottime_prev = boottime_now;
        return RESUME_NOT_DETECTED;
    }

    int64_t delta_monotonic = timespec_diff_ns(&monotonic_prev, &monotonic_now);
    int64_t delta_boottime = timespec_diff_ns(&boottime_prev, &boottime_now);

    monotonic_prev = monotonic_now;
    boottime_prev = boottime_now;

    return delta_boottime > delta_monotonic + THRESHOLD_NS
               ? RESUME_DETECTED
               : RESUME_NOT_DETECTED;
}

static void fscanf_ignore_sensor(FILE *f, long pos) {
    char sensor_name[SENSOR_NAME_MAX];
    int ret = fscanf(f, "ignore_sensor %255s ", sensor_name);
    if (ret == 1) {
        expect(num_to_ignore_sensors < MAX_IGNORED_SENSORS);
        snprintf(ignored_sensors_arr[num_to_ignore_sensors], SENSOR_NAME_MAX,
                 "%s", sensor_name);
        num_to_ignore_sensors++;
    } else {
        expect(fseek(f, pos, SEEK_SET) == 0);
    }
}

static bool read_sensor_file(DIR *sensor_dir, const char *file_name, char *buf,
                             size_t buf_size) {
    int fd = openat(dirfd(sensor_dir), file_name, O_RDONLY);
    if (fd < 0)
        return false;
    ssize_t len = read(fd, buf, buf_size - 1);
    close(fd);
    if (len <= 0)
        return false;
    buf[len] = '\0';
    buf[strcspn(buf, "\n")] = '\0';
    return true;
}

static bool is_sensor_name_ignored(DIR *sensor_dir) {
    char sensor_name[SENSOR_NAME_MAX];
    if (!read_sensor_file(sensor_dir, "name", sensor_name, sizeof(sensor_name)))
        return false;
    for (size_t i = 0; i < num_to_ignore_sensors; i++) {
        if (strcmp(sensor_name, ignored_sensors_arr[i]) == 0)
            return true;
    }
    return false;
}

static int full_speed_supported(void) {
    FILE *f = fopen(FAN_CONTROL_FILE, "re");
    char line[256]; // If exceeded, we'll just read again
    int found = 0;

    expect(f);

    while (fgets(line, sizeof(line), f) != NULL) {
        if (strstr(line, "full-speed") != NULL) {
            found = 1;
            break;
        }
    }

    fclose(f);
    return found;
}

static bool is_cpu_driver(const char *sensor_name) {
    return strstr(sensor_name, "cpu") != NULL ||
           strcmp(sensor_name, "coretemp") == 0 ||
           strcmp(sensor_name, "k10temp") == 0 ||
           strcmp(sensor_name, "zenpower") == 0;
}

/* Die/package readings (coretemp "Package id N"/"Physical id N", k10temp
 * "Tctl"/"Tdie"/"Tccd") react faster and hotter than the ACPI/EC sensor, so
 * they are deliberately excluded from fan control: letting them drive the
 * control temperature made the fan thrash on bursty load. */
static bool is_die_label(const char *label) {
    return strncmp(label, "Package ", strlen("Package ")) == 0 ||
           strncmp(label, "Physical id ", strlen("Physical id ")) == 0 ||
           strncmp(label, "Tctl", strlen("Tctl")) == 0 ||
           strncmp(label, "Tdie", strlen("Tdie")) == 0 ||
           strncmp(label, "Tccd", strlen("Tccd")) == 0;
}

/* The ACPI/EC CPU-level reading (label "CPU" on the thinkpad EC): the same
 * value the platform's critical-shutdown trip reacts to. */
static bool is_cpu_label(const char *label) {
    return strncmp(label, "CPU", strlen("CPU")) == 0;
}

static enum SensorKind get_sensor_kind(DIR *sensor_dir,
                                       const struct dirent *sensor_file,
                                       bool cpu_driver) {
    char label_file[NAME_MAX + sizeof("_label")];
    char label[SENSOR_NAME_MAX];
    /* hwmon temp files are tempN_input with a matching tempN_label; strip the
     * _input suffix before appending _label. */
    size_t base_len = strlen(sensor_file->d_name) - strlen("_input");
    int ret = snprintf(label_file, sizeof(label_file), "%.*s_label",
                       (int)base_len, sensor_file->d_name);
    if (ret >= 0 && (size_t)ret < sizeof(label_file) &&
        read_sensor_file(sensor_dir, label_file, label, sizeof(label))) {
        if (strncmp(label, "Core ", strlen("Core ")) == 0)
            return SENSOR_CPU_CORE;
        if (is_die_label(label))
            return SENSOR_OTHER;
        if (is_cpu_label(label))
            return SENSOR_CPU;
    }

    return cpu_driver ? SENSOR_CPU : SENSOR_OTHER;
}

/* Repeated identical scan failures are logged only on the first occurrence
 * (or when the failing stage or errno changes), so a persistent failure does
 * not write a journal line every second. */
static void log_scan_failure(const char *stage, int failure_errno) {
    static const char *last_stage;
    static int last_errno;
    if (last_stage != stage || last_errno != failure_errno) {
        err("hwmon scan %s failed: %s\n", stage, strerror(failure_errno));
        last_stage = stage;
        last_errno = failure_errno;
    }
}

/* Handle a single tempN_input entry from a hwmon device directory. */
static void add_sensor_fd(struct SensorSet *set, DIR *sensor_dir,
                          const char *hwmon_name, bool cpu_driver,
                          const struct dirent *sensor_file) {
    if (strncmp(sensor_file->d_name, "temp", 4) != 0 ||
        !strstr(sensor_file->d_name, "_input"))
        return;
    if (set->num_sensor_fds >= MAX_SENSOR_FDS) {
        /* Rather than aborting a running daemon, treat the scan as
         * unusable; the active set is kept instead. */
        set->complete = false;
        return;
    }
    int temp_fd = openat(dirfd(sensor_dir), sensor_file->d_name, O_RDONLY);
    if (temp_fd < 0) {
        /* The device may have gone away mid-scan. */
        set->complete = false;
        return;
    }
    struct stat st;
    if (fstat(temp_fd, &st) < 0) {
        close(temp_fd);
        set->complete = false;
        return;
    }
    enum SensorKind kind = get_sensor_kind(sensor_dir, sensor_file, cpu_driver);
    struct Sensor *sensor = set->sensors + set->num_sensor_fds++;
    *sensor = (struct Sensor){
        .fd = temp_fd, .kind = kind, .dev = st.st_dev, .ino = st.st_ino};
    int ret = snprintf(sensor->path, sizeof(sensor->path), "%s/%s", hwmon_name,
                       sensor_file->d_name);
    expect(ret >= 0 && (size_t)ret < sizeof(sensor->path));
    if (kind == SENSOR_CPU_CORE)
        set->num_cpu_core_sensors++;
    if (kind != SENSOR_OTHER)
        set->num_cpu_temp_sensors++;
}

static void add_sensor_fds(struct SensorSet *set, DIR *sensor_dir,
                           const char *hwmon_name, bool cpu_driver) {
    for (;;) {
        errno = 0;
        struct dirent *sensor_file = readdir(sensor_dir);
        if (!sensor_file) {
            if (errno != 0) {
                /* An I/O error here means entries after this point were
                 * not seen: the scan is partial. */
                log_scan_failure("readdir", errno);
                set->complete = false;
            }
            break;
        }
        add_sensor_fd(set, sensor_dir, hwmon_name, cpu_driver, sensor_file);
    }
}

static int compare_sensor(const void *a, const void *b) {
    return strcmp(((const struct Sensor *)a)->path,
                  ((const struct Sensor *)b)->path);
}

static void scan_hwmon_dir(struct SensorSet *set, DIR *hwmon_dir,
                           const struct dirent *hwmon_entry) {
    int sensor_dir_fd =
        openat(dirfd(hwmon_dir), hwmon_entry->d_name, O_RDONLY | O_DIRECTORY);
    if (sensor_dir_fd < 0) {
        /* The device may have gone away mid-scan. */
        set->complete = false;
        return;
    }
    DIR *sensor_dir = fdopendir(sensor_dir_fd);
    if (!sensor_dir) {
        close(sensor_dir_fd);
        set->complete = false;
        return;
    }
    char sensor_name[SENSOR_NAME_MAX];
    bool cpu_driver = read_sensor_file(sensor_dir, "name", sensor_name,
                                       sizeof(sensor_name)) &&
                      is_cpu_driver(sensor_name);
    if (is_sensor_name_ignored(sensor_dir)) {
        set->num_ignored_sensors++;
        closedir(sensor_dir);
        return;
    }
    add_sensor_fds(set, sensor_dir, hwmon_entry->d_name, cpu_driver);
    closedir(sensor_dir);
}

/* Scan hwmon_root into set. Returns true only when the scan completed and
 * the snapshot is usable; callers must not swap in a set from a failed or
 * partial scan. On first-tick failures this exits via exit_if_first_tick(). */
static bool populate_sensor_fds(struct SensorSet *set) {
    memset(set, 0, sizeof(*set));
    set->complete = true;

    int hwmon_fd = open(hwmon_root, O_RDONLY | O_DIRECTORY);
    if (hwmon_fd < 0) {
        log_scan_failure("open", errno);
        exit_if_first_tick();
        return false;
    }
    DIR *hwmon_dir = fdopendir(hwmon_fd);
    if (!hwmon_dir) {
        log_scan_failure("fdopendir", errno);
        close(hwmon_fd);
        exit_if_first_tick();
        return false;
    }

    for (;;) {
        errno = 0;
        struct dirent *hwmon_entry = readdir(hwmon_dir);
        if (!hwmon_entry) {
            if (errno != 0) {
                log_scan_failure("readdir", errno);
                set->complete = false;
            }
            break;
        }
        scan_hwmon_dir(set, hwmon_dir, hwmon_entry);
    }
    closedir(hwmon_dir);

    /* readdir() order is not guaranteed to be stable between scans, so sort by
     * path to make sensor set comparison deterministic. */
    qsort(set->sensors, set->num_sensor_fds, sizeof(struct Sensor),
          compare_sensor);
    return set->complete;
}

static void close_sensor_fds(struct SensorSet *set) {
    for (size_t i = 0; i < set->num_sensor_fds; i++) {
        close(set->sensors[i].fd);
    }
}

/* True when the two sensor sets are not element-wise identical. The per-kind
 * counters are derived from the sensor list, so comparing the count and the
 * per-sensor identity is sufficient. */
static bool sensor_sets_differ(const struct SensorSet *a,
                               const struct SensorSet *b) {
    if (a->num_sensor_fds != b->num_sensor_fds)
        return true;
    for (size_t i = 0; i < a->num_sensor_fds; i++) {
        if (a->sensors[i].kind != b->sensors[i].kind ||
            a->sensors[i].dev != b->sensors[i].dev ||
            a->sensors[i].ino != b->sensors[i].ino ||
            strcmp(a->sensors[i].path, b->sensors[i].path) != 0)
            return true;
    }
    return false;
}

/* Some hwmon drivers register late: for example, coretemp is autoloaded by
 * udev and can appear well after we have started. Re-scan every tick and swap
 * in the new set when it differs from the active one. */
static void refresh_sensors(void) {
    struct SensorSet *cur = &sensor_set;
    struct SensorSet *scratch = &sensor_set_scratch;

    if (populate_sensor_fds(scratch) && sensor_sets_differ(scratch, cur)) {
        close_sensor_fds(cur);
        *cur = *scratch;
        info(
            "Sensor set changed: %zu sensors (%zu CPU core, %zu non-core CPU)\n",
            cur->num_sensor_fds, cur->num_cpu_core_sensors,
            cur->num_cpu_temp_sensors - cur->num_cpu_core_sensors);
    } else {
        /* Failed or partial scan, or nothing changed: discard the scratch
         * set and keep the active one. */
        close_sensor_fds(scratch);
    }
}

/* The kernel supports reading new values without reopening the FD */
static int read_temp_fd(int fd) {
    char buf[32];
    if (lseek(fd, 0, SEEK_SET) < 0)
        return TEMP_INVALID;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (n <= 0)
        return TEMP_INVALID;
    buf[n] = '\0';
    int val;
    return (sscanf(buf, "%d", &val) == 1) ? val : TEMP_INVALID;
}

static int get_average_temp(void) {
    int64_t temp_sum = 0;
    size_t num_valid_temps = 0;
    int cpu_max = TEMP_INVALID;
    enum SensorKind selected_kind = SENSOR_OTHER;
    if (sensor_set.num_cpu_core_sensors > 0)
        selected_kind = SENSOR_CPU_CORE;
    else if (sensor_set.num_cpu_temp_sensors > 0)
        selected_kind = SENSOR_CPU;

    for (size_t i = 0; i < sensor_set.num_sensor_fds; i++) {
        int temp = read_temp_fd(sensor_set.sensors[i].fd);
        if (temp == TEMP_INVALID || temp <= 0)
            continue;
        /* Track the ACPI/EC sensor reading: the one the platform's
         * critical-shutdown trip reacts to. Die/package readings are
         * excluded (see is_die_label): they spike hotter and faster than
         * the trip sensor. */
        if (sensor_set.sensors[i].kind == SENSOR_CPU && temp > cpu_max)
            cpu_max = temp;
        if (selected_kind != SENSOR_OTHER &&
            sensor_set.sensors[i].kind != selected_kind)
            continue;
        temp_sum += temp;
        num_valid_temps++;
    }

    if (num_valid_temps == 0 && cpu_max == TEMP_INVALID) {
        err("Couldn't find any valid temperature\n");
        exit_if_first_tick();
        return TEMP_INVALID;
    }

    /* Fan-control temperature: the maximum of the core average and the
     * ACPI/EC sensor reading. */
    int average_temp = TEMP_INVALID;
    if (num_valid_temps > 0)
        average_temp = MILLIC_TO_C((int)(temp_sum / (int64_t)num_valid_temps));
    if (cpu_max != TEMP_INVALID)
        cpu_max = MILLIC_TO_C(cpu_max);
    return cpu_max > average_temp ? cpu_max : average_temp;
}

#define write_fan_level(level) write_fan("level", level)

static int write_fan(const char *command, const char *value) {
    FILE *f = fopen(FAN_CONTROL_FILE, "we");
    int ret;

    if (!f) {
        err("%s: fopen: %s%s\n", FAN_CONTROL_FILE, strerror(errno),
            errno == ENOENT ? " (is thinkpad_acpi loaded?)" : "");
        exit_if_first_tick();
        return -errno;
    }

    expect(setvbuf(f, NULL, _IONBF, 0) == 0); /* Make fprintf see errors */
    ret = fprintf(f, "%s %s", command, value);
    if (ret < 0) {
        err("%s: write: %s%s\n", FAN_CONTROL_FILE, strerror(errno),
            errno == EINVAL ? " (did you enable fan_control=1?)" : "");
        exit_if_first_tick();
        fclose(f);
        return -errno;
    }
    expect(clock_gettime(CLOCK_MONOTONIC_COARSE, &last_watchdog_ping) == 0);
    fclose(f);
    return 0;
}

static void write_watchdog_timeout(const time_t timeout) {
    char timeout_s[sizeof(S_DEFAULT_WATCHDOG_SECS)]; /* max timeout value */
    int ret =
        snprintf(timeout_s, sizeof(timeout_s), "%" PRIuMAX, (uintmax_t)timeout);
    expect(ret >= 0 && (size_t)ret < sizeof(timeout_s));
    write_fan("watchdog", timeout_s);
}

enum set_fan_status {
    FAN_LEVEL_NOT_SET,
    FAN_LEVEL_SET,
    FAN_LEVEL_INVALID,
};

static enum set_fan_status set_fan_level(void) {
    int average_temp = get_average_temp(), temp_penalty = 0;
    static unsigned int tick_penalty = tick_hysteresis;

    if (tick_penalty > 0) {
        tick_penalty--;
    }

    if (average_temp == TEMP_INVALID) {
        write_fan_level("full-speed");
        return FAN_LEVEL_INVALID;
    }

    /* Panic: at 95C we are only a few degrees below the firmware's critical
     * trip point (typically 98C-105C on ThinkPads, fed by the ACPI/EC
     * sensor), so engage maximum immediately rather than waiting out the
     * debounce. */
    if (average_temp >= 95 && current_rule != rules + FAN_MAX) {
        const struct Rule *rule = rules + FAN_MAX;
        current_rule = rule;
        tick_penalty = tick_hysteresis;
        printf("[FAN] Temperature now %dC, at or above panic threshold 95C, "
               "fan set to %s\n",
               average_temp, rule->name);
        write_fan_level(rule->tpacpi_level);
        return FAN_LEVEL_SET;
    }

    for (size_t i = 0; i < FAN_INVALID; i++) {
        if (average_temp > rules[i].threshold)
            level_ticks[i]++;
        else
            level_ticks[i] = 0;
    }

    for (size_t i = 0; i < FAN_INVALID; i++) {
        const struct Rule *rule = rules + i;

        if (rule == current_rule) {
            if (tick_penalty) {
                // Must wait longer until able to move down levels
                return FAN_LEVEL_NOT_SET;
            }
            temp_penalty = temp_hysteresis;
        }

        if (rule->threshold < temp_penalty ||
            (rule->threshold - temp_penalty) < average_temp) {
            if (rule != current_rule) {
                bool moving_up = current_rule == NULL || rule < current_rule;
                if (moving_up &&
                    level_ticks[i] < (unsigned int)rule->debounce_secs) {
                    // Must stay above the threshold for debounce_secs before
                    // engaging a higher fan level
                    return FAN_LEVEL_NOT_SET;
                }
                current_rule = rule;
                tick_penalty = tick_hysteresis;
                printf("[FAN] Average temperature now %dC, fan set to %s\n",
                       average_temp, rule->name);
                write_fan_level(rule->tpacpi_level);
                return FAN_LEVEL_SET;
            }
            return FAN_LEVEL_NOT_SET;
        }
    }

    err("No threshold matched?\n");
    return FAN_LEVEL_INVALID;
}

#define WATCHDOG_GRACE_PERIOD_SECS 2
static void maybe_ping_watchdog(void) {
    struct timespec now;

    expect(current_rule);
    expect(clock_gettime(CLOCK_MONOTONIC_COARSE, &now) == 0);

    if (detect_suspend() == RESUME_DETECTED) {
        // On resume, some models need a manual fan write again, or they will
        // revert to "auto".
        info("Clock jump detected, possible resume. Rewriting fan level\n");
        write_fan_level(current_rule->tpacpi_level);
    }

    if (now.tv_sec - last_watchdog_ping.tv_sec <
        (watchdog_secs - WATCHDOG_GRACE_PERIOD_SECS)) {
        return;
    }

    // Transitioning from level 0 -> level 0 can cause a brief fan spinup on
    // some models, so don't reset the timer by write_fan_level().
    write_watchdog_timeout(watchdog_secs);
}

#define CONFIG_PATH "/etc/zcfan.conf"
#define fscanf_int_for_key(f, pos, name, dest)                                 \
    do {                                                                       \
        int val;                                                               \
        if (fscanf(f, name " %d ", &val) == 1) {                               \
            dest = val;                                                        \
        } else {                                                               \
            expect(fseek(f, pos, SEEK_SET) == 0);                              \
        }                                                                      \
    } while (0)

#define fscanf_str_for_key(f, pos, name, dest)                                 \
    do {                                                                       \
        char val[CONFIG_MAX_STRLEN + 1];                                       \
        if (fscanf(f, name " %" S_CONFIG_MAX_STRLEN "s ", val) == 1) {         \
            strncpy(dest, val, CONFIG_MAX_STRLEN);                             \
            dest[CONFIG_MAX_STRLEN] = '\0';                                    \
        } else {                                                               \
            expect(fseek(f, pos, SEEK_SET) == 0);                              \
        }                                                                      \
    } while (0)

static void get_config(void) {
    FILE *f;

    f = fopen(CONFIG_PATH, "re");
    if (!f) {
        if (errno != ENOENT) {
            err("%s: fopen: %s\n", CONFIG_PATH, strerror(errno));
            exit_if_first_tick();
        }
        return;
    }

    while (!feof(f)) {
        long pos = ftell(f);
        int ch;
        expect(pos >= 0);
        fscanf_int_for_key(f, pos, "max_temp", rules[FAN_MAX].threshold);
        fscanf_int_for_key(f, pos, "med_temp", rules[FAN_MED].threshold);
        fscanf_int_for_key(f, pos, "low_temp", rules[FAN_LOW].threshold);
        fscanf_int_for_key(f, pos, "max_debounce_secs",
                           rules[FAN_MAX].debounce_secs);
        fscanf_int_for_key(f, pos, "med_debounce_secs",
                           rules[FAN_MED].debounce_secs);
        fscanf_int_for_key(f, pos, "low_debounce_secs",
                           rules[FAN_LOW].debounce_secs);
        fscanf_int_for_key(f, pos, "watchdog_secs", watchdog_secs);
        fscanf_int_for_key(f, pos, "temp_hysteresis", temp_hysteresis);
        fscanf_str_for_key(f, pos, "max_level", rules[FAN_MAX].tpacpi_level);
        fscanf_str_for_key(f, pos, "med_level", rules[FAN_MED].tpacpi_level);
        fscanf_str_for_key(f, pos, "low_level", rules[FAN_LOW].tpacpi_level);
        fscanf_ignore_sensor(f, pos);
        if (ftell(f) == pos) {
            while ((ch = fgetc(f)) != EOF && ch != '\n') {}
        }
    }

    for (size_t i = 0; i < FAN_OFF; i++) {
        if (rules[i].debounce_secs < 0)
            rules[i].debounce_secs = 0;
    }

    /* Maximum value handled by the kernel is 120, and
     * (watchdog_secs - WATCHDOG_GRACE_PERIOD_SECS) must stay positive. */
    if (watchdog_secs < WATCHDOG_GRACE_PERIOD_SECS ||
        watchdog_secs > DEFAULT_WATCHDOG_SECS) {
        err("%s: value for the watchdog_secs directive has to be between %d and %d\n",
            CONFIG_PATH, WATCHDOG_GRACE_PERIOD_SECS, DEFAULT_WATCHDOG_SECS);
        exit(1);
    }

    fclose(f);
}

static void print_thresholds(void) {
    for (size_t i = 0; i < FAN_OFF; i++) {
        const struct Rule *rule = rules + i;
        printf("[CFG] At %dC fan is set to %s (after %ds above threshold)\n",
               rule->threshold, rule->name, rule->debounce_secs);
    }
    if (sensor_set.num_cpu_core_sensors > 0) {
        printf("[CFG] Averaging %zu CPU core sensors\n",
               sensor_set.num_cpu_core_sensors);
    } else if (sensor_set.num_cpu_temp_sensors > 0) {
        printf("[CFG] Averaging %zu CPU sensors\n",
               sensor_set.num_cpu_temp_sensors);
    } else {
        printf("[CFG] Averaging all %zu temperature sensors\n",
               sensor_set.num_sensor_fds);
    }
    printf("[CFG] Ignored %zu present sensors based on config\n",
           sensor_set.num_ignored_sensors);
}

static void stop(int sig) {
    (void)sig;
    run = 0;
}

static void handle_sigpwr(int sig) {
    (void)sig;
    pending_sleep = 1;
}

static void handle_sigusr2(int sig) {
    (void)sig;
    pending_resume = 1;
}

int main(int argc, char *argv[]) {
    const struct sigaction sa_exit = {
        .sa_handler = stop,
    };

    (void)argv;

    if (argc != 1) {
        printf("zcfan: Zero-configuration ThinkPad fan daemon.\n\n");
        printf("  [any argument]     Show this help\n\n");
        printf("See the zcfan(1) man page for details.\n");
        return 0;
    }

    get_config();
    expect(sigaction(SIGTERM, &sa_exit, NULL) == 0);
    expect(sigaction(SIGINT, &sa_exit, NULL) == 0);
    expect(sigaction(SIGPWR,
                     &(const struct sigaction){.sa_handler = handle_sigpwr},
                     NULL) == 0);
    expect(sigaction(SIGUSR2,
                     &(const struct sigaction){.sa_handler = handle_sigusr2},
                     NULL) == 0);

    expect(setvbuf(stdout, output_buf, _IOLBF, sizeof(output_buf)) == 0);

    if (!full_speed_supported()) {
        err("level \"full-speed\" not supported, using level 7\n");
        strncpy(rules[FAN_MAX].tpacpi_level, "7", CONFIG_MAX_STRLEN);
        rules[FAN_MAX].tpacpi_level[CONFIG_MAX_STRLEN] = '\0';
    }

    write_watchdog_timeout(watchdog_secs);
    if (!populate_sensor_fds(&sensor_set)) {
        /* First-tick open failures exit inside the scan; anything else that
         * lands here (readdir error, sensor count overflow) means we could
         * not obtain a trustworthy sensor list. */
        return 1;
    }
    print_thresholds();

    int fan_control_enabled = 1;

    while (run) {
        refresh_sensors();
        if (fan_control_enabled) {
            enum set_fan_status set = set_fan_level();
            if (set != FAN_LEVEL_SET) {
                maybe_ping_watchdog();
            }
        }
        if (run) {
            sleep(1);
            first_tick = 0;
        }
        if (pending_sleep) {
            pending_sleep = 0;
            info("Fan control disabled for sleep\n");
            if (write_fan_level("auto") == 0)
                write_watchdog_timeout(0);
            fan_control_enabled = 0;
        }
        if (pending_resume) {
            pending_resume = 0;
            info("Fan control enabled for resume\n");
            fan_control_enabled = 1;
            expect(current_rule);
            write_fan_level(current_rule->tpacpi_level);
            write_watchdog_timeout(watchdog_secs);
        }
    }

    printf("[FAN] Quit requested, reenabling thinkpad_acpi fan control\n");
    if (write_fan_level("auto") == 0) {
        write_watchdog_timeout(0);
    }
    close_sensor_fds(&sensor_set);
    return 0;
}
