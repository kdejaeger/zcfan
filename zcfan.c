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
/* Overridable so tests can run against a synthetic fan-control file. */
static const char *fan_control_file = FAN_CONTROL_FILE;
#define TEMP_INVALID INT_MIN
#define TEMP_MIN (INT_MIN + 1)
#define NS_IN_SEC 1000000000L         // 1 second in nanoseconds
#define RESUME_THRESHOLD_NS 200000000 // 0.2 seconds

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

#define MAX_IGNORED_SENSOR_NAMES 1024
#define SENSOR_NAME_MAX 256
static char ignored_sensor_names[MAX_IGNORED_SENSOR_NAMES][SENSOR_NAME_MAX];
static size_t num_ignored_sensor_names = 0;

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
    /* Primary control inputs (EC and per-core readings). If a set has
     * none, every readable sensor feeds the average as a last resort. */
    size_t num_control_sensors;
    /* Per-core die readings among the control sensors. */
    size_t num_core_sensors;
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
/* Ticks a freshly engaged level is held before it may move again. */
static const unsigned int hold_ticks = 3;
/* From this temperature the firmware's critical-shutdown trip is close
 * enough that the fan must not wait out the debounce, and must not be
 * reduced while the reading persists (see set_fan_level()). */
#define FAN_PANIC_TEMP_C 95
static char output_buf[512];
static const struct Rule *current_rule = NULL;
/* True while temperature acquisition failed for this tick and the
 * invalid-temperature fail-safe commands full-speed. While active, the
 * runaway healing and the held-level reassertions must not fight the
 * fail-safe: rewriting the held level would undo the full-speed command,
 * and the fan spinning under the fail-safe must not be read as a
 * runaway. */
static bool fail_safe_temps = false;
/* True while the control temperature is at or above the panic threshold
 * this tick. Like the fail-safe flag it is refreshed by set_fan_level()
 * before the watchdog path runs in the same tick, and it keeps runaway
 * healing from restoring a held level underneath the panic response. */
static bool panic_temps = false;

/* The level to keep on the wire: the held level, or the maximum level
 * while the fail-safe or the panic response is in charge (both write the
 * configured maximum; a failed engagement write leaves the bookkeeping at
 * the old rule, so the reassertions must not undo the response). Callers
 * must ensure current_rule is non-NULL. */
static const char *held_level(void) {
    return fail_safe_temps || panic_temps ? rules[FAN_MAX].tpacpi_level
                                          : current_rule->tpacpi_level;
}
static unsigned int level_ticks[FAN_INVALID];
static volatile sig_atomic_t run = 1;
static volatile sig_atomic_t pending_sleep = 0;
static volatile sig_atomic_t pending_resume = 0;
static bool first_tick = true; /* Stop running if errors are immediate */

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

    return delta_boottime > delta_monotonic + RESUME_THRESHOLD_NS
               ? RESUME_DETECTED
               : RESUME_NOT_DETECTED;
}

static void fscanf_ignore_sensor(FILE *f, long pos) {
    char sensor_name[SENSOR_NAME_MAX];
    int ret = fscanf(f, "ignore_sensor %255s ", sensor_name);
    if (ret == 1) {
        expect(num_ignored_sensor_names < MAX_IGNORED_SENSOR_NAMES);
        snprintf(ignored_sensor_names[num_ignored_sensor_names],
                 SENSOR_NAME_MAX, "%s", sensor_name);
        num_ignored_sensor_names++;
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
    for (size_t i = 0; i < num_ignored_sensor_names; i++) {
        if (strcmp(sensor_name, ignored_sensor_names[i]) == 0)
            return true;
    }
    return false;
}

static bool full_speed_supported(void) {
    FILE *f = fopen(fan_control_file, "re");
    char line[256]; // If exceeded, we'll just read again
    bool found = false;

    expect(f);

    while (fgets(line, sizeof(line), f) != NULL) {
        if (strstr(line, "full-speed") != NULL) {
            found = true;
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
        set->num_core_sensors++;
    if (kind != SENSOR_OTHER)
        set->num_control_sensors++;
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
            cur->num_sensor_fds, cur->num_core_sensors,
            cur->num_control_sensors - cur->num_core_sensors);
    } else {
        /* Failed or partial scan, or nothing changed: discard the scratch
         * set and keep the active one. */
        close_sensor_fds(scratch);
    }
}

/* Parse a leading decimal integer like sscanf("%d")/atoi, but reject
 * values outside int range: an out-of-range conversion is undefined
 * behavior, and a faulty sensor must degrade to "unreadable" rather than
 * produce garbage. Trailing text is ignored, as the old parsers did. */
static bool parse_int_range(const char *buf, int *out) {
    errno = 0;
    char *end;
    long val = strtol(buf, &end, 10);
    if (end == buf || errno == ERANGE || val < INT_MIN || val > INT_MAX)
        return false;
    *out = (int)val;
    return true;
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
    return parse_int_range(buf, &val) ? val : TEMP_INVALID;
}

/* The two fan-control inputs, read once per tick. average_temp is the mean
 * of the preferred readable readings and decides level reductions: the
 * ACPI/EC reading lags the die and must not keep the fan spinning after the
 * cores have cooled. control_temp folds in the hottest CPU-level reading and
 * decides level engagement, the debounce accounting, and the panic path: it
 * is what the platform's critical-shutdown trip reacts to. */
struct FanTemps {
    int average_temp;
    int control_temp;
    /* True only when average_temp is a genuine per-core average. Without
     * one there is no die-vs-EC lag to compensate for, so reductions
     * follow the control temperature: average_temp is set equal to it
     * when readable, and set_fan_level() falls back to it when the core
     * average is unreadable. */
    bool average_is_core;
};

static struct FanTemps get_fan_temps(void) {
    int64_t temp_sum = 0;
    size_t num_valid_temps = 0;
    int cpu_max = TEMP_INVALID;
    /* Averaging prefers CPU-core readings, then other CPU-level readings,
     * then everything: the preference is decided per tick from VALID
     * readings, not discovery counts, so a discovered-but-unreadable
     * preferred class falls through to readable inputs instead of
     * degenerating into the invalid-temperature fail-safe. Die/package
     * readings are only averaged when no CPU-level reading exists (see
     * is_die_label): they spike hotter and faster than the trip sensor. */
    static const enum SensorKind preference[] = {
        SENSOR_CPU_CORE,
        SENSOR_CPU,
        SENSOR_OTHER,
    };
    enum SensorKind selected_kind = SENSOR_OTHER;
    for (size_t p = 0; p < sizeof(preference) / sizeof(*preference); p++) {
        selected_kind = preference[p];
        temp_sum = 0;
        num_valid_temps = 0;
        for (size_t i = 0; i < sensor_set.num_sensor_fds; i++) {
            int temp = read_temp_fd(sensor_set.sensors[i].fd);
            if (temp == TEMP_INVALID || temp <= 0)
                continue;
            /* Track the hottest CPU-level reading: that is the class of
             * reading the platform's critical-shutdown trip reacts to
             * (the ACPI/EC sensor on most ThinkPads). Die/package readings
             * are excluded (see is_die_label): they spike hotter and
             * faster than the trip sensor. */
            if (sensor_set.sensors[i].kind == SENSOR_CPU && temp > cpu_max)
                cpu_max = temp;
            if (selected_kind != SENSOR_OTHER &&
                sensor_set.sensors[i].kind != selected_kind)
                continue;
            temp_sum += temp;
            num_valid_temps++;
        }
        if (num_valid_temps > 0)
            break;
    }

    struct FanTemps temps = {TEMP_INVALID, TEMP_INVALID, false};
    /* Log only the transition into (and out of) a total outage: a
     * persistent sensor failure would otherwise log once per second. */
    static bool logged_no_valid_temps = false;
    if (num_valid_temps == 0 && cpu_max == TEMP_INVALID) {
        /* The full-speed write itself is repeated on purpose: it is the
         * safety floor. */
        if (!logged_no_valid_temps) {
            err("Couldn't find any valid temperature\n");
            logged_no_valid_temps = true;
        }
        exit_if_first_tick();
        return temps;
    }
    logged_no_valid_temps = false;

    if (num_valid_temps > 0)
        temps.average_temp =
            MILLIC_TO_C((int)(temp_sum / (int64_t)num_valid_temps));
    if (cpu_max != TEMP_INVALID)
        cpu_max = MILLIC_TO_C(cpu_max);
    temps.control_temp =
        cpu_max > temps.average_temp ? cpu_max : temps.average_temp;
    temps.average_is_core =
        selected_kind == SENSOR_CPU_CORE && num_valid_temps > 0;
    /* Without a genuine core average there is no die-vs-EC lag to
     * compensate for, so reductions follow the control temperature: the
     * hottest available readings drive both directions. */
    if (!temps.average_is_core && temps.average_temp != TEMP_INVALID)
        temps.average_temp = temps.control_temp;
    return temps;
}

#define write_fan_level(level) write_fan("level", level)

static int write_fan(const char *command, const char *value) {
    FILE *f = fopen(fan_control_file, "we");
    int ret;

    if (!f) {
        err("%s: fopen: %s%s\n", fan_control_file, strerror(errno),
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

/* Transition logs are labeled by the input that decided the level: the
 * ACPI/EC control reading when raising and whenever no core average drove
 * the decision; the core average for ordinary reductions. */
static const char *fan_source(bool moving_up, bool average_is_core) {
    return moving_up || !average_is_core ? "Temperature" : "Core average";
}

static enum set_fan_status set_fan_level(void) {
    struct FanTemps temps = get_fan_temps();
    int control_temp = temps.control_temp;
    int threshold_discount = 0;
    static unsigned int hold_ticks_remaining = hold_ticks;

    if (hold_ticks_remaining > 0) {
        hold_ticks_remaining--;
    }

    if (control_temp == TEMP_INVALID) {
        fail_safe_temps = true;
        /* The configured maximum level is the fail-safe floor: it has
         * passed config validation (never "0"/"auto") and the "7"
         * fallback, so this write also works on kernels without
         * full-speed support. */
        write_fan_level(rules[FAN_MAX].tpacpi_level);
        return FAN_LEVEL_INVALID;
    }
    panic_temps = control_temp >= FAN_PANIC_TEMP_C;
    if (fail_safe_temps) {
        /* Recovering from the invalid-temperature fail-safe: the wire
         * holds the maximum level while the bookkeeping still holds a
         * level that would never be re-written on its own. Restore it
         * before re-evaluating thresholds -- unless the recovered
         * temperature already demands the maximum (panic band), where the
         * panic write below takes the wire directly. */
        if (current_rule && !panic_temps) {
            if (write_fan_level(current_rule->tpacpi_level) != 0)
                /* Failed restore: keep the fail-safe active so the next
                 * tick retries instead of trusting bookkeeping that no
                 * longer matches the wire. */
                return FAN_LEVEL_NOT_SET;
        }
        fail_safe_temps = false;
    }

    /* Level reductions follow the core average alone: the ACPI/EC reading
     * lags the die and must not keep the fan spinning after the cores have
     * cooled. If the core average is unreadable the ACPI/EC reading drives
     * both directions. The panic band needs no exception here: maximum is
     * held outright at or above the band threshold (see below), independent
     * of any configured thresholds. */
    int reduction_temp = temps.average_temp;
    if (reduction_temp == TEMP_INVALID)
        reduction_temp = control_temp;

    /* Panic: at 95C we are only a few degrees below the firmware's critical
     * trip point (typically 98C-105C on ThinkPads, fed by the ACPI/EC
     * sensor), so engage maximum immediately rather than waiting out the
     * debounce, and never reduce while the reading persists: a custom
     * threshold can sit above the band, so the hold must not rely on
     * discounted thresholds. */
    if (control_temp >= FAN_PANIC_TEMP_C) {
        if (current_rule == rules + FAN_MAX)
            return FAN_LEVEL_NOT_SET;
        const struct Rule *rule = rules + FAN_MAX;
        /* Advance the recorded level only on a successful write: a failed
         * panic write must leave the old rule in place, so the next tick
         * retries instead of assuming maximum is already on the wire. */
        if (write_fan_level(rule->tpacpi_level) != 0)
            return FAN_LEVEL_NOT_SET;
        current_rule = rule;
        hold_ticks_remaining = hold_ticks;
        printf("[FAN] Temperature now %dC, at or above panic threshold %dC, "
               "fan set to %s\n",
               control_temp, FAN_PANIC_TEMP_C, rule->name);
        return FAN_LEVEL_SET;
    }

    for (size_t i = 0; i < FAN_INVALID; i++) {
        if (control_temp > rules[i].threshold)
            level_ticks[i]++;
        else
            level_ticks[i] = 0;
    }

    for (size_t i = 0; i < FAN_INVALID; i++) {
        const struct Rule *rule = rules + i;
        /* Engagement follows the control temperature; reductions the core
         * average. */
        bool moving_up = current_rule == NULL || rule < current_rule;
        int rule_temp = moving_up ? control_temp : reduction_temp;

        if (rule == current_rule) {
            if (hold_ticks_remaining) {
                // A freshly engaged level is held before it may move again
                return FAN_LEVEL_NOT_SET;
            }
            /* Thresholds at or below the current level are discounted by
             * the hysteresis, so the fan does not flap between adjacent
             * levels when the reading hovers near a threshold. */
            threshold_discount = temp_hysteresis;
        }

        /* Discounting TEMP_MIN would overflow, so the floor matches via the
         * first clause. */
        if (rule->threshold < threshold_discount ||
            (rule->threshold - threshold_discount) < rule_temp) {
            if (rule != current_rule) {
                if (moving_up &&
                    level_ticks[i] < (unsigned int)rule->debounce_secs) {
                    if (current_rule == NULL) {
                        // Must stay above the threshold for debounce_secs
                        // before engaging a higher fan level
                        return FAN_LEVEL_NOT_SET;
                    }
                    /* A pending raise must not block reductions: keep
                     * scanning so lower rules can still be evaluated
                     * against the core average. */
                    continue;
                }
                /* A reduction must not carry stale up-debounce credit: the
                 * departed level re-arms only after a fresh observation, so
                 * a control temperature that lingers above its threshold
                 * (the ACPI/EC reading decaying after a burst) cannot
                 * re-engage it instantly. */
                if (write_fan_level(rule->tpacpi_level) != 0)
                    /* Same as the panic path: only a successful write may
                     * advance the recorded level. */
                    return FAN_LEVEL_NOT_SET;
                if (!moving_up)
                    memset(level_ticks, 0, sizeof(level_ticks));
                current_rule = rule;
                hold_ticks_remaining = hold_ticks;
                printf("[FAN] %s now %dC, fan set to %s\n",
                       fan_source(moving_up, temps.average_is_core), rule_temp,
                       rule->name);
                return FAN_LEVEL_SET;
            }
            return FAN_LEVEL_NOT_SET;
        }
    }

    err("No threshold matched?\n");
    return FAN_LEVEL_INVALID;
}

#define WATCHDOG_GRACE_PERIOD_SECS 2
/* The EC can quietly drop manual fan control and resume its own automatic
 * management. Two distinct failure shapes have been observed:
 *
 * 1. The status line reads "enabled" while zcfan holds a level: the EC took
 *    over as a whole. Re-assert the level.
 *
 * 2. On a dual-fan model (X1 Carbon Gen 14, kernel 7.2.6): the status line
 *    still reads "disabled" (manual control held), zcfan commands level 0,
 *    one fan obeys and stops, but the other keeps spinning at a steady
 *    speed for hours at cool temperatures. The EC is running its own
 *    policy on that fan alone while still reporting manual control, so a
 *    status re-assert would never fire. An excursion to the maximum
 *    fan level (the configured max_level, "full-speed" by default)
 *    followed by the held level has been verified on hardware to clear
 *    the stuck per-fan state. */

/* Ticks spent above this RPM while the held level is off, for two
 * consecutive watchdog refreshes, count as the per-fan runaway. */
#define FAN_SELF_RUNNING_RPM 500
/* A commanded fan needs a moment to spin down: require two consecutive
 * refreshes before treating a spinning fan as runaway. */
#define FAN_RUNAWAY_CONFIRM_TICKS 2
/* Full-speed excursion attempts per runaway episode, spaced
 * FAN_RUNAWAY_RETRY_TICKS + 1 watchdog refreshes apart (at the default
 * watchdog_secs of 120, about 12 minutes); after the attempts are spent
 * the episode is abandoned and logged until the fan stops. */
#define FAN_RUNAWAY_MAX_EXCURSIONS 3
#define FAN_RUNAWAY_RETRY_TICKS 5

/* Read a sysfs attribute file at an absolute path. */
static bool read_sensor_file_path(const char *path, char *buf,
                                  size_t buf_size) {
    int fd = open(path, O_RDONLY);
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

/* Highest RPM among the fan inputs of the thinkpad hwmon device, or 0 when
 * none can be read. A single-fan model simply lacks fan2_input. */
static int max_fan_rpm(void) {
    int max_rpm = 0;
    DIR *fans_dir = opendir(hwmon_root);
    if (!fans_dir)
        return 0;
    const struct dirent *entry;
    while ((entry = readdir(fans_dir)) != NULL) {
        if (strncmp(entry->d_name, "hwmon", 5) != 0)
            continue;
        char path[80];
        int ret = snprintf(path, sizeof(path), "%s/%s/name", hwmon_root,
                           entry->d_name);
        if (ret < 0 || (size_t)ret >= sizeof(path))
            continue;
        char driver_name[SENSOR_NAME_MAX];
        if (!read_sensor_file_path(path, driver_name, sizeof(driver_name)) ||
            strcmp(driver_name, "thinkpad") != 0)
            continue;
        for (int fan_num = 1; fan_num <= 2; fan_num++) {
            ret = snprintf(path, sizeof(path), "%s/%s/fan%d_input", hwmon_root,
                           entry->d_name, fan_num);
            if (ret < 0 || (size_t)ret >= sizeof(path))
                continue;
            char rpm_buf[16];
            if (!read_sensor_file_path(path, rpm_buf, sizeof(rpm_buf)))
                continue;
            int rpm;
            if (!parse_int_range(rpm_buf, &rpm))
                continue;
            if (rpm > max_rpm)
                max_rpm = rpm;
        }
    }
    closedir(fans_dir);
    return max_rpm;
}

/* Detect an uncommanded spinning fan and clear it with an excursion to
 * the maximum fan level. Returns true only when a complete excursion
 * (maximum level written, then the held level restored) succeeded.
 *
 * Called once per watchdog refresh. Any level above off has a legitimate
 * spin speed above the runaway threshold, so detection is meaningful only
 * while the off level is held; when another level is held, confirmation
 * and retry spacing are dropped, but the attempt budget and the
 * exhausted-hold survive: the stuck per-fan EC state is not expected to
 * heal itself, and a reset budget would let temperature oscillation
 * around a level threshold drive endless excursion loops. The excursion
 * writes the configured max_level ("full-speed" by default), which has
 * been verified on hardware to clear the EC's stuck per-fan state.
 *
 * While the invalid-temperature fail-safe holds (fail_safe_temps) or the
 * panic band is active (panic_temps), healing is suspended entirely: the
 * fail-safe/panic response commands the maximum level, so a spinning fan
 * is expected, and restoring the held off level would fight the response. */
static bool maybe_fix_runaway_fan(void) {
    /* Consecutive refreshes with a spinning fan while level 0 is held.
     * Reset when no fan spins or the held level is not off. */
    static int runaways_seen = 0;
    /* Excursion attempts left in the current runaway episode. */
    static unsigned int attempts_left = FAN_RUNAWAY_MAX_EXCURSIONS;
    /* Watchdog refreshes until the next excursion attempt is allowed. */
    static unsigned int retry_ticks = 0;
    /* Set when the episode is exhausted; cleared when the runaway stops. */
    static bool hold_until_stop = false;

    if (current_rule != rules + FAN_OFF || fail_safe_temps || panic_temps) {
        /* Confirmation and retry spacing are only meaningful while the
         * off level is held, with no fail-safe in charge and no panic
         * response active. */
        runaways_seen = 0;
        retry_ticks = 0;
        return false;
    }

    int max_rpm = max_fan_rpm();
    if (max_rpm <= FAN_SELF_RUNNING_RPM) {
        /* No fan spins beyond the commanded stop: clean state. */
        runaways_seen = 0;
        attempts_left = FAN_RUNAWAY_MAX_EXCURSIONS;
        retry_ticks = 0;
        if (hold_until_stop) {
            hold_until_stop = false;
            info("Fan no longer self-running; runaway handling re-armed\n");
        }
        return false;
    }

    /* A fan spins beyond the commanded speed. Require confirmation across
     * two consecutive refreshes before acting: a freshly commanded fan
     * needs a moment to spin down before it reads as stopped. */
    runaways_seen++;
    if (runaways_seen < FAN_RUNAWAY_CONFIRM_TICKS)
        return false;
    if (hold_until_stop)
        return false;
    if (attempts_left == 0) {
        hold_until_stop = true;
        err("Fan still self-running at the maximum level after %d "
            "excursion(s); leaving it alone until it stops\n",
            FAN_RUNAWAY_MAX_EXCURSIONS);
        return false;
    }
    if (retry_ticks > 0) {
        retry_ticks--;
        return false;
    }

    /* Runaway confirmed: clear the EC's per-fan state with a full-speed
     * excursion (verified on hardware to reset the fan to the commanded
     * level), then restore the held level. */
    printf("[FAN] Fan self-running at %d RPM while level %s is commanded "
           "(%u previous excursion(s))\n",
           max_rpm, current_rule->name,
           (unsigned)(FAN_RUNAWAY_MAX_EXCURSIONS - attempts_left));
    int rc = write_fan("level", rules[FAN_MAX].tpacpi_level);
    if (rc != 0) {
        /* The excursion write failed: without it a restore would leave
         * the EC in unknown territory, so back off without consuming an
         * attempt and try again after the retry spacing. */
        err("Fan runaway excursion write failed: %s\n", strerror(-rc));
        retry_ticks = FAN_RUNAWAY_RETRY_TICKS;
        return false;
    }
    /* Keep the excursion on the wire briefly before re-asserting, so
     * the EC is guaranteed to see it; the daemon blocks for this
     * second, which is safe: the fan is at the excursion level
     * meanwhile. Retry if a signal interrupts the pause: restoring
     * early could leave the excursion unseen by the EC. */
    struct timespec pause = {1, 0};
    while (nanosleep(&pause, &pause) == -1 && errno == EINTR) {
        /* Suspend arrived mid-excursion: the EC loses manual state anyway,
         * so finish promptly instead of spending the remaining pause past
         * the sleep service's one-second wait. */
        if (pending_sleep)
            break;
    }
    rc = write_fan_level(current_rule->tpacpi_level);
    if (rc != 0) {
        /* The excursion level is on the wire, but the held level could
         * not be restored. Treat the attempt as not spent: the fan still
         * reads far above the commanded level, so after the retry
         * spacing the whole excursion-and-restore sequence runs again.
         * Consuming the attempt here could abandon the fan at the
         * excursion level once the budget is spent. */
        err("Fan runaway restore write failed: %s\n", strerror(-rc));
        retry_ticks = FAN_RUNAWAY_RETRY_TICKS;
        return false;
    }
    attempts_left--;
    retry_ticks = FAN_RUNAWAY_RETRY_TICKS;
    return true;
}

static void reassert_fan_control(void) {
    /* Detect and heal the per-fan runaway (failure shape 2) before the
     * status check: with the stuck state the status line keeps reading
     * "disabled", so only the fan speeds reveal the problem. */
    maybe_fix_runaway_fan();

    FILE *f = fopen(fan_control_file, "re");
    char line[128];
    char status[sizeof("disabled")];

    if (!f)
        return;
    while (fgets(line, sizeof(line), f) != NULL) {
        if (sscanf(line, "status: %8s", status) == 1 &&
            strcmp(status, "enabled") == 0) {
            printf("[FAN] EC reverted to automatic fan control, re-asserting "
                   "%s\n",
                   current_rule->name);
            write_fan_level(held_level());
            break;
        }
    }
    fclose(f);
}

static void maybe_ping_watchdog(void) {
    struct timespec now;

    expect(clock_gettime(CLOCK_MONOTONIC_COARSE, &now) == 0);

    /* On the first ticks no rule has been engaged yet: a hot start can sit
     * above a threshold waiting out its debounce with current_rule still
     * NULL. There is no level to rewrite on resume detection in that case;
     * the next control tick engages one and writes it. The watchdog itself
     * is still maintained (see below): nothing manual has been written, so
     * the EC is in its automatic mode either way. */
    if (detect_suspend() == RESUME_DETECTED) {
        // On resume, some models need a manual fan write again, or they will
        // revert to "auto".
        if (current_rule) {
            info("Clock jump detected, possible resume. Rewriting fan level\n");
            write_fan_level(held_level());
        }
    }

    if (now.tv_sec - last_watchdog_ping.tv_sec <
        (watchdog_secs - WATCHDOG_GRACE_PERIOD_SECS)) {
        return;
    }

    /* With no rule engaged the EC is in its automatic mode (we have not
     * written a level yet), so there is nothing to re-assert. */
    if (current_rule)
        reassert_fan_control();

    // Transitioning from level 0 -> level 0 can cause a brief fan spinup on
    // some models, so don't reset the timer by write_fan_level().
    write_watchdog_timeout(watchdog_secs);
}

#define CONFIG_PATH "/etc/zcfan.conf"
/* Overridable so tests can point at a synthetic config file. */
static const char *config_path = CONFIG_PATH;
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

    f = fopen(config_path, "re");
    if (!f) {
        if (errno != ENOENT) {
            err("%s: fopen: %s\n", config_path, strerror(errno));
            exit_if_first_tick();
        }
        return;
    }

    while (!feof(f)) {
        long pos = ftell(f);
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
            int ch;
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
            config_path, WATCHDOG_GRACE_PERIOD_SECS, DEFAULT_WATCHDOG_SECS);
        exit(1);
    }

    /* "0" and "auto" would make the panic path and the runaway excursion
     * command the fan to stop or hand control back to the EC exactly when
     * the machine is hottest. */
    if (strcmp(rules[FAN_MAX].tpacpi_level, "0") == 0 ||
        strcmp(rules[FAN_MAX].tpacpi_level, "auto") == 0) {
        err("%s: max_level \"%s\" would defeat the panic and runaway-safety paths\n",
            config_path, rules[FAN_MAX].tpacpi_level);
        exit(1);
    }

    /* The threshold discount is subtracted from rule thresholds: a
     * negative hysteresis raises reduced thresholds, and an enormous one
     * overflows the subtraction. More than 100C is nonsense anyway. */
    if (temp_hysteresis < 0 || temp_hysteresis > 100) {
        err("%s: value for the temp_hysteresis directive has to be between 0 and 100\n",
            config_path);
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
    if (sensor_set.num_core_sensors > 0) {
        printf("[CFG] Averaging %zu CPU core sensors\n",
               sensor_set.num_core_sensors);
    } else if (sensor_set.num_control_sensors > 0) {
        printf("[CFG] Averaging %zu CPU sensors\n",
               sensor_set.num_control_sensors);
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

    if (!full_speed_supported() &&
        strcmp(rules[FAN_MAX].tpacpi_level, "full-speed") == 0) {
        /* Only the default level is replaced when full-speed is not
         * supported: an explicitly configured max_level is the user's
         * choice and is not silently overridden. */
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

    bool fan_control_enabled = true;

    while (run) {
        refresh_sensors();
        if (fan_control_enabled) {
            enum set_fan_status status = set_fan_level();
            if (status != FAN_LEVEL_SET) {
                maybe_ping_watchdog();
            }
        }
        if (run) {
            sleep(1);
            first_tick = false;
        }
        if (pending_sleep) {
            pending_sleep = 0;
            info("Fan control disabled for sleep\n");
            /* Deliberate exception to the invalid-temperature fail-safe:
             * suspend hands thermal management to the EC's own automatic
             * mode, which stays in charge while zcfan is not controlling. */
            if (write_fan_level("auto") == 0)
                write_watchdog_timeout(0);
            fan_control_enabled = false;
        }
        if (pending_resume) {
            pending_resume = 0;
            info("Fan control enabled for resume\n");
            fan_control_enabled = true;
            /* current_rule can still be NULL if we resumed during the
             * startup debounce window; the next control tick engages a
             * level and writes it. */
            if (current_rule)
                write_fan_level(held_level());
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
