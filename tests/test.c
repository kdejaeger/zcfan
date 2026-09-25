/* Functional tests for the sensor discovery and refresh logic in zcfan.c.
 *
 * The daemon source is compiled in directly (with main renamed), and the
 * hwmon sysfs tree is faked with regular files in a temporary directory, so
 * the tests need no privileges and no real hardware. */

#define main zcfan_unused_main
#include "../zcfan.c"
#undef main

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>

static int num_failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("FAIL: %s (line %d)\n", #cond, __LINE__);                   \
            num_failures++;                                                    \
        }                                                                      \
    } while (0)

static char fixture_root[] = "/tmp/zcfan-test-XXXXXX";

static void rm_rf(const char *path);

/* Test failures abort via expect(); clean the fixture up first so failed
 * runs do not leave directories behind in /tmp. The daemon's own expect()
 * calls inside the included zcfan.c keep their original definition. */
static void test_expect_failure(int cond, const char *expr, int line) {
    if (cond)
        return;
    rm_rf(fixture_root);
    fprintf(stderr, "FATAL: !(%s) at tests/test.c:%d\n", expr, line);
    abort();
}
#undef expect
#define expect(x) test_expect_failure(!!(x), #x, __LINE__)

static void write_file(const char *path, const char *content) {
    FILE *f = fopen(path, "w");
    expect(f);
    fprintf(f, "%s\n", content);
    expect(fclose(f) == 0);
}

static void make_hwmon(const char *name, const char *driver_name) {
    char path[512];
    expect(snprintf(path, sizeof(path), "%s/%s", fixture_root, name) > 0);
    expect(mkdir(path, 0755) == 0);
    char name_path[600];
    expect(snprintf(name_path, sizeof(name_path), "%s/name", path) > 0);
    write_file(name_path, driver_name);
}

static void make_temp(const char *hwmon, const char *temp, const char *label,
                      long millidegrees) {
    char path[600];
    expect(snprintf(path, sizeof(path), "%s/%s/%s", fixture_root, hwmon, temp) >
           0);
    char value[32];
    expect(snprintf(value, sizeof(value), "%ld", millidegrees) > 0);
    write_file(path, value);
    if (label != NULL) {
        char label_path[600];
        size_t base_len = strlen(temp) - strlen("_input");
        expect(snprintf(label_path, sizeof(label_path), "%s/%s/%.*s_label",
                        fixture_root, hwmon, (int)base_len, temp) > 0);
        write_file(label_path, label);
    }
}

static struct Sensor *sensor_for_path(const char *path) {
    for (size_t i = 0; i < sensor_set.num_sensor_fds; i++) {
        if (strcmp(sensor_set.sensors[i].path, path) == 0)
            return &sensor_set.sensors[i];
    }
    return NULL;
}

/* Convenience accessor for the control temperature from get_fan_temps(). */
static int control_temp(void) { return get_fan_temps().control_temp; }

/* Redirect stdout to a fixture file so [FAN] log lines can be asserted on.
 * Capture windows must contain no CHECK calls: CHECK reports failures via
 * stdout. Callers restore the returned stream before asserting. */
static FILE *capture_fan_log(const char *path) {
    fflush(stdout);
    FILE *saved_stdout = stdout;
    stdout = fopen(path, "w");
    expect(stdout != NULL);
    return saved_stdout;
}

static void read_fan_log(const char *path, FILE *saved_stdout, char *line,
                         size_t size) {
    fflush(stdout);
    expect(fclose(stdout) == 0);
    stdout = saved_stdout;
    FILE *log = fopen(path, "re");
    expect(log != NULL);
    expect(fgets(line, (int)size, log) != NULL);
    char extra[4];
    expect(fgets(extra, (int)sizeof(extra), log) == NULL); /* one line only */
    expect(fclose(log) == 0);
}

/* Same capture pattern as capture_fan_log()/read_fan_log(), but for stderr,
 * where info()/err() write. Only expect() failures write to stderr besides
 * the code under test, and those abort anyway, so a captured window holds
 * exactly the line(s) the code under test emitted. */
static FILE *capture_stderr_log(const char *path) {
    fflush(stderr);
    FILE *saved_stderr = stderr;
    stderr = fopen(path, "w");
    expect(stderr != NULL);
    return saved_stderr;
}

static void read_stderr_log(const char *path, FILE *saved_stderr, char *line,
                            size_t size) {
    fflush(stderr);
    expect(fclose(stderr) == 0);
    stderr = saved_stderr;
    FILE *log = fopen(path, "re");
    expect(log != NULL);
    expect(fgets(line, (int)size, log) != NULL);
    char extra[4];
    expect(fgets(extra, (int)sizeof(extra), log) == NULL); /* one line only */
    expect(fclose(log) == 0);
}

/* Read a whole (small) file, e.g. the fan-control fixture, to assert the
 * exact command the code under test last wrote. */
static void read_text_file(const char *path, char *buf, size_t size) {
    FILE *f = fopen(path, "re");
    expect(f != NULL);
    size_t n = fread(buf, 1, size - 1, f);
    buf[n] = '\0';
    expect(fclose(f) == 0);
}

/* Capture window that must stay silent: assert the captured stderr held
 * nothing at all (read_stderr_log would FATAL on an empty window). */
static void expect_stderr_empty(const char *path, FILE *saved_stderr) {
    fflush(stderr);
    expect(fclose(stderr) == 0);
    stderr = saved_stderr;
    FILE *log = fopen(path, "re");
    expect(log != NULL);
    expect(fgetc(log) == EOF);
    expect(fclose(log) == 0);
}

/* get_config() exits on invalid directives, which cannot be asserted in
 * this process: fork a child, run the parse there, and require exit
 * status 1. The child's parse mutates only its own copy of the rules. */
static void expect_config_status(const char *content, int expected_status) {
    char config_file[640];
    expect(snprintf(config_file, sizeof(config_file), "%s/parse-test.conf",
                    fixture_root) > 0);
    write_file(config_file, content);
    fflush(stdout);
    fflush(stderr);
    pid_t pid = fork();
    if (pid == 0) {
        expect(freopen("/dev/null", "w", stderr) != NULL);
        config_path = config_file;
        get_config();
        _exit(0); /* unreachable for every rejected configuration */
    }
    expect(pid > 0);
    int status;
    expect(waitpid(pid, &status, 0) == pid);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == expected_status);
}

static void rm_rf(const char *path) {
    DIR *dir = opendir(path);
    if (dir != NULL) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (strcmp(entry->d_name, ".") == 0 ||
                strcmp(entry->d_name, "..") == 0)
                continue;
            char child[600];
            expect(snprintf(child, sizeof(child), "%s/%s", path,
                            entry->d_name) > 0);
            if (unlink(child) != 0 && errno == EISDIR)
                rm_rf(child);
        }
        closedir(dir);
    }
    rmdir(path);
}

int main(void) {
    expect(mkdtemp(fixture_root) != NULL);
    hwmon_root = fixture_root;

    /* hwmon0: coretemp with per-core and package readings; hwmon1: thinkpad
     * pseudo-sensors; hwmon2: an unlabelled reading. */
    make_hwmon("hwmon0", "coretemp");
    make_temp("hwmon0", "temp1_input", "Package id 0", 45000);
    make_temp("hwmon0", "temp2_input", "Core 0", 50000);
    make_temp("hwmon0", "temp3_input", "Core 1", 52000);
    make_temp("hwmon0", "temp4_input", "Core 2", 54000);
    make_hwmon("hwmon1", "thinkpad");
    make_temp("hwmon1", "temp1_input", "CPU", 48000);
    make_temp("hwmon1", "temp2_input", "GPU", 40000);
    make_hwmon("hwmon2", "nvme");
    make_temp("hwmon2", "temp1_input", NULL, 41000);

    /* Initial scan: classification and averaging. */
    CHECK(populate_sensor_fds(&sensor_set));
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(sensor_set.num_core_sensors == 3);
    CHECK(sensor_set.num_control_sensors == 4);
    CHECK(sensor_set.num_ignored_sensors == 0);
    CHECK(sensor_for_path("hwmon0/temp2_input")->kind == SENSOR_CPU_CORE);
    /* Die/package reading: excluded from control. */
    CHECK(sensor_for_path("hwmon0/temp1_input")->kind == SENSOR_OTHER);
    CHECK(sensor_for_path("hwmon1/temp1_input")->kind == SENSOR_CPU);
    CHECK(sensor_for_path("hwmon2/temp1_input")->kind == SENSOR_OTHER);
    for (size_t i = 1; i < sensor_set.num_sensor_fds; i++) {
        CHECK(strcmp(sensor_set.sensors[i - 1].path,
                     sensor_set.sensors[i].path) < 0);
    }
    CHECK(control_temp() == 52); /* mean of the three cores */

    /* Unchanged refresh: silent, and the active fds are kept. */
    int fd_before = sensor_for_path("hwmon0/temp2_input")->fd;
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(sensor_for_path("hwmon0/temp2_input")->fd == fd_before);

    /* Late driver registration (the boot race this defends against). */
    make_hwmon("hwmon3", "coretemp");
    make_temp("hwmon3", "temp1_input", "Core 9", 60000);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 8);
    CHECK(sensor_set.num_core_sensors == 4);
    CHECK(control_temp() == 54);
    CHECK(sensor_for_path("hwmon3/temp1_input")->kind == SENSOR_CPU_CORE);

    /* Removal is picked up again. */
    char path[600];
    expect(snprintf(path, sizeof(path), "%s/hwmon3/temp1_input", fixture_root) >
           0);
    expect(unlink(path) == 0);
    expect(snprintf(path, sizeof(path), "%s/hwmon3/temp1_label", fixture_root) >
           0);
    expect(unlink(path) == 0);
    expect(snprintf(path, sizeof(path), "%s/hwmon3/name", fixture_root) > 0);
    expect(unlink(path) == 0);
    expect(snprintf(path, sizeof(path), "%s/hwmon3", fixture_root) > 0);
    expect(rmdir(path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(sensor_set.num_core_sensors == 3);
    CHECK(control_temp() == 52);

    /* Driver reload into the same hwmonN: identical path and kind, but a new
     * inode. A stale fd would keep reading the pre-replacement content. */
    ino_t ino_before = sensor_for_path("hwmon0/temp2_input")->ino;
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp2_input", fixture_root) >
           0);
    char tmp_path[640];
    expect(snprintf(tmp_path, sizeof(tmp_path), "%s.having-a-nice-day", path) >
           0);
    write_file(tmp_path, "99000");
    expect(rename(tmp_path, path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(sensor_set.num_core_sensors == 3);
    CHECK(sensor_for_path("hwmon0/temp2_input")->ino != ino_before);
    CHECK(control_temp() == 68); /* 99000 replaces 50000 in the mean */

    /* An unreadable input makes the scan incomplete: the partial snapshot
     * must be discarded and the active set retained. */
    expect(snprintf(path, sizeof(path), "%s/hwmon2/temp9_input", fixture_root) >
           0);
    expect(symlink("/nonexistent-target", path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(control_temp() == 68);
    expect(unlink(path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(control_temp() == 68);

    /* A hwmon device that vanishes mid-scan (here: dangling symlink): its
     * directory open fails, the scan is incomplete, and the active set must
     * be retained. */
    expect(snprintf(path, sizeof(path), "%s/hwmon9", fixture_root) > 0);
    expect(symlink("/nonexistent-target", path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(control_temp() == 68);
    expect(unlink(path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(control_temp() == 68);

    /* A failing hwmon root must not crash the daemon (this exits on the
     * first tick; first_tick is cleared here to model steady state). */
    first_tick = false;
    hwmon_root = "/nonexistent-zcfan-test";
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(control_temp() == 68);
    hwmon_root = fixture_root;
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(control_temp() == 68);

    /* The effective temperature is the maximum of the core average and the
     * ACPI/EC sensor reading: that sensor is what the firmware's critical
     * shutdown trip reacts to. */
    expect(snprintf(path, sizeof(path), "%s/hwmon1/temp1_input", fixture_root) >
           0);
    write_file(path, "95000");
    CHECK(control_temp() == 95); /* ACPI/EC sensor wins over the average */
    write_file(path, "60000");
    CHECK(control_temp() == 68); /* 68C average beats 60C ACPI/EC */

    /* A die/package spike must not move the control temperature: hwmon0's
     * "Package id 0" is deliberately excluded from fan control. */
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp1_input", fixture_root) >
           0);
    write_file(path, "99000");
    CHECK(control_temp() == 68); /* die reading ignored */

    /* Every excluded die label: they classify as ordinary sensors even on
     * CPU temperature drivers, so no label variant can leak into control. */
    make_temp("hwmon0", "temp5_input", "Tdie", 99000);
    make_temp("hwmon0", "temp6_input", "Physical id 1", 99000);
    make_temp("hwmon1", "temp3_input", "Tctl", 99000);
    make_temp("hwmon1", "temp4_input", "Tccd", 99000);
    refresh_sensors();
    CHECK(sensor_for_path("hwmon0/temp5_input")->kind == SENSOR_OTHER);
    CHECK(sensor_for_path("hwmon0/temp6_input")->kind == SENSOR_OTHER);
    CHECK(sensor_for_path("hwmon1/temp3_input")->kind == SENSOR_OTHER);
    CHECK(sensor_for_path("hwmon1/temp4_input")->kind == SENSOR_OTHER);
    CHECK(control_temp() == 68); /* die spikes at 99C ignored */

    /* With every averaged reading invalid (0) and die-labelled readings
     * classified out, the core class yields no valid reading: the
     * preference must fall through to the CPU class, whose ACPI/EC sensor
     * (60000) provides the average instead of erroring. */
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp1_input", fixture_root) >
           0);
    write_file(path, "0");
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp2_input", fixture_root) >
           0);
    write_file(path, "0");
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp3_input", fixture_root) >
           0);
    write_file(path, "0");
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp4_input", fixture_root) >
           0);
    write_file(path, "0");
    CHECK(control_temp() == 60); /* hwmon1's ACPI/EC sensor survives */

    /* With the CPU-level reading invalid too, the preference must fall
     * through to the readable ordinary sensors instead of reporting no
     * valid temperature: the die-labelled readings dominate the fallback
     * average, as documented for the no-CPU-sensor case. */
    expect(snprintf(path, sizeof(path), "%s/hwmon1/temp1_input", fixture_root) >
           0);
    write_file(path, "0");
    CHECK(control_temp() == 79); /* GPU 40 + nvme 41 + die 99x4, averaged */
    /* Out-of-range sensor text must degrade to unreadable, not overflow:
     * the nvme reading leaves the fallback average (87C without it). */
    expect(snprintf(path, sizeof(path), "%s/hwmon2/temp1_input", fixture_root) >
           0);
    write_file(path, "99999999999999999999");
    CHECK(control_temp() == 87); /* overflow reading rejected */
    write_file(path, "-99999999999999999999");
    CHECK(control_temp() == 87); /* negative overflow rejected too */
    write_file(path, "41000");
    expect(snprintf(path, sizeof(path), "%s/hwmon1/temp1_input", fixture_root) >
           0);
    write_file(path, "60000");
    CHECK(control_temp() == 60); /* CPU-level reading preferred again */

    /* The 95C panic path engages maximum immediately, bypassing debounce:
     * from FAN_OFF with no debounce ticks accrued, the level still moves.
     * The fan-control file is a fixture from here on, so level writes
     * succeed and the recorded level genuinely advances. */
    char fan_control_fixture[640];
    expect(snprintf(fan_control_fixture, sizeof(fan_control_fixture),
                    "%s/fan-control", fixture_root) > 0);
    write_file(fan_control_fixture, "");
    fan_control_file = fan_control_fixture;
    expect(snprintf(path, sizeof(path), "%s/hwmon1/temp1_input", fixture_root) >
           0);
    write_file(path, "95000");
    current_rule = rules + FAN_OFF;
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_MAX);

    /* The recorded level must advance only on a successful write: a failed
     * panic write leaves the old rule in place so the next tick retries
     * instead of assuming maximum is already on the wire. */
    fan_control_file = "/nonexistent-zcfan-fan-control";
    write_file(path, "96000"); /* path: hwmon1/temp1_input */
    current_rule = rules + FAN_OFF;
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* failed write: retryable */
    CHECK(current_rule == rules + FAN_OFF);
    fan_control_file = fan_control_fixture;
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* retried on the next tick */
    CHECK(current_rule == rules + FAN_MAX);

    /* Same guard for ordinary (non-panic) engagement: a failed reduction
     * write leaves the recorded level alone so the next tick retries. */
    fan_control_file = "/nonexistent-zcfan-fan-control";
    make_temp("hwmon0", "temp2_input", NULL, 45000);
    make_temp("hwmon0", "temp3_input", NULL, 47000);
    make_temp("hwmon0", "temp4_input", NULL, 49000);
    make_temp("hwmon1", "temp1_input", NULL, 65000);
    for (int i = 0; i < 3; i++) {
        CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* hold, then fail */
        CHECK(current_rule == rules + FAN_MAX);
    }
    fan_control_file = fan_control_fixture;
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* retried and engaged */
    CHECK(current_rule == rules + FAN_OFF);

    /* A NULL current_rule (first ticks still waiting out a debounce on a
     * hot start) must not abort in the watchdog path. */
    current_rule = NULL;
    maybe_ping_watchdog();
    CHECK(current_rule == NULL);

    /* End to end for that hot-start sequence: an effective temperature in
     * the maximum level's debounce window keeps current_rule NULL through
     * set_fan_level() and the watchdog call that follows it in main(). */
    write_file(path, "91000"); /* path: hwmon1/temp1_input */
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET);
    CHECK(current_rule == NULL);
    maybe_ping_watchdog();
    CHECK(current_rule == NULL);

    /* Reductions follow the core average alone. Engage maximum via the
     * ACPI/EC sensor, then hold the EC reading at 75C (above the low
     * threshold) while the cores cool: the fan must still step down and
     * off. Per-call assertions guard the whole trajectory, so a masked
     * intermediate transition cannot pass. */
    make_temp("hwmon1", "temp1_input", NULL, 95000);
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_MAX);

    make_temp("hwmon1", "temp1_input", NULL, 75000);
    make_temp("hwmon0", "temp2_input", NULL, 55000);
    make_temp("hwmon0", "temp3_input", NULL, 57000);
    make_temp("hwmon0", "temp4_input", NULL, 59000);
    const struct Rule *reduction_path[] = {rules + FAN_MAX, rules + FAN_MAX,
                                           rules + FAN_LOW, rules + FAN_LOW,
                                           rules + FAN_LOW};
    for (int i = 0; i < 5; i++) {
        const struct Rule *before = current_rule;
        enum set_fan_status st = set_fan_level();
        CHECK(current_rule == reduction_path[i]);
        /* SET exactly when the level changed. */
        CHECK(st ==
              (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
    }

    make_temp("hwmon0", "temp2_input", NULL, 45000);
    make_temp("hwmon0", "temp3_input", NULL, 47000);
    make_temp("hwmon0", "temp4_input", NULL, 49000);
    const struct Rule *off_path[] = {rules + FAN_OFF, rules + FAN_OFF,
                                     rules + FAN_OFF, rules + FAN_OFF,
                                     rules + FAN_OFF};
    for (int i = 0; i < 5; i++) {
        const struct Rule *before = current_rule;
        enum set_fan_status st = set_fan_level();
        CHECK(current_rule == off_path[i]);
        /* SET exactly when the level changed. */
        CHECK(st ==
              (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
    }

    /* In the panic band maximum must be held outright, even at the band's
     * exact lower edge (95C), although the cores are cool. */
    make_temp("hwmon1", "temp1_input", NULL, 95000);
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_MAX);
    for (int i = 0; i < 5; i++) {
        const struct Rule *before = current_rule;
        enum set_fan_status st = set_fan_level();
        CHECK(current_rule == rules + FAN_MAX);
        /* SET exactly when the level changed. */
        CHECK(st ==
              (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
    }

    /* Leaving the panic band resumes core-average reductions: a 75C control
     * temperature with a 47C core average steps straight from maximum to
     * off. */
    make_temp("hwmon1", "temp1_input", NULL, 75000);
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_OFF);

    /* With the core average unreadable the ACPI/EC reading drives both
     * directions: 95C panics to maximum and a falling EC reading reduces
     * again. */
    make_temp("hwmon0", "temp2_input", NULL, 0);
    make_temp("hwmon0", "temp3_input", NULL, 0);
    make_temp("hwmon0", "temp4_input", NULL, 0);
    make_temp("hwmon1", "temp1_input", NULL, 95000);
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_MAX);
    make_temp("hwmon1", "temp1_input", NULL, 65000);
    const struct Rule *ec_path[] = {rules + FAN_MAX, rules + FAN_MAX,
                                    rules + FAN_MED, rules + FAN_MED,
                                    rules + FAN_MED};
    for (int i = 0; i < 5; i++) {
        const struct Rule *before = current_rule;
        enum set_fan_status st = set_fan_level();
        CHECK(current_rule == ec_path[i]);
        /* SET exactly when the level changed. */
        CHECK(st ==
              (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
    }
    make_temp("hwmon1", "temp1_input", NULL, 45000);
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_OFF);

    /* A pending raise must not block a reduction: with the fan at low, a
     * 47C core average forces off even while a medium raise (EC 85C) is
     * still waiting out its debounce. */
    memset(level_ticks, 0, sizeof(level_ticks));
    make_temp("hwmon0", "temp2_input", NULL, 45000);
    make_temp("hwmon0", "temp3_input", NULL, 47000);
    make_temp("hwmon0", "temp4_input", NULL, 49000);
    make_temp("hwmon1", "temp1_input", NULL, 85000);
    current_rule = rules + FAN_LOW;
    const struct Rule *bypass_path[] = {rules + FAN_LOW, rules + FAN_LOW,
                                        rules + FAN_OFF, rules + FAN_OFF,
                                        rules + FAN_OFF};
    for (int i = 0; i < 5; i++) {
        const struct Rule *before = current_rule;
        enum set_fan_status st = set_fan_level();
        CHECK(current_rule == bypass_path[i]);
        /* SET exactly when the level changed. */
        CHECK(st ==
              (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
    }

    /* The bypass applies to every pending upward candidate: with the fan
     * at medium, a 55C core average reduces to low even though a maximum
     * raise (EC 94C) is still waiting out its debounce. */
    memset(level_ticks, 0, sizeof(level_ticks));
    make_temp("hwmon0", "temp2_input", NULL, 45000);
    make_temp("hwmon0", "temp3_input", NULL, 55000);
    make_temp("hwmon0", "temp4_input", NULL, 65000);
    make_temp("hwmon1", "temp1_input", NULL, 94000);
    current_rule = rules + FAN_MED;
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* reduces despite pending max */
    CHECK(current_rule == rules + FAN_LOW);
    for (int i = 0; i < 4; i++) {
        CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* low holds */
        CHECK(current_rule == rules + FAN_LOW);
    }

    /* Control-driven first engagement: from a clean state a 75C control
     * temperature (cool cores, EC 75C) engages low after its full 60-tick
     * debounce. */
    memset(level_ticks, 0, sizeof(level_ticks));
    current_rule = NULL;
    make_temp("hwmon0", "temp2_input", NULL, 45000);
    make_temp("hwmon0", "temp3_input", NULL, 47000);
    make_temp("hwmon0", "temp4_input", NULL, 49000);
    make_temp("hwmon1", "temp1_input", NULL, 75000);
    for (int i = 0; i < 59; i++) {
        CHECK(set_fan_level() == FAN_LEVEL_NOT_SET);
        CHECK(current_rule == NULL);
    }
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_LOW);

    /* Unit: transition labels follow the deciding input. */
    CHECK(strcmp(fan_source(true, true), "Temperature") == 0);
    CHECK(strcmp(fan_source(false, true), "Core average") == 0);
    CHECK(strcmp(fan_source(false, false), "Temperature") == 0);

    /* Stale up-debounce credit must not re-engage a departed level: while
     * the EC reading holds at 85C, medium's counter is already full when
     * medium is left, so the reduction must clear it and medium may only
     * re-engage on the 30th fresh tick. */
    memset(level_ticks, 0, sizeof(level_ticks));
    current_rule = NULL;
    make_temp("hwmon1", "temp1_input", NULL, 85000);
    make_temp("hwmon0", "temp2_input", NULL, 52000);
    make_temp("hwmon0", "temp3_input", NULL, 55000);
    make_temp("hwmon0", "temp4_input", NULL, 58000);
    for (int i = 0; i < 29; i++) {
        CHECK(set_fan_level() == FAN_LEVEL_NOT_SET);
        CHECK(current_rule == NULL);
    }
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_MED);
    const struct Rule *stale_path[] = {
        rules + FAN_MED, rules + FAN_MED, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW, rules + FAN_LOW,
        rules + FAN_MED};
    for (int i = 0; i < 33; i++) {
        const struct Rule *before = current_rule;
        enum set_fan_status st = set_fan_level();
        CHECK(current_rule == stale_path[i]);
        /* SET exactly when the level changed. */
        CHECK(st ==
              (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
    }

    /* Integration: the labels must reach the real transition log. The
     * capture windows contain no CHECKs; each window's third call reduces
     * (the first two drain the freshly-engaged hold and print nothing). */
    char log_path[640];
    expect(snprintf(log_path, sizeof(log_path), "%s/fan-log", fixture_root) >
           0);
    /* stderr capture buffers, shared by the runaway and fail-safe tests */
    char err_path[640];
    char err_line[256];
    expect(snprintf(err_path, sizeof(err_path), "%s/err-log", fixture_root) >
           0);
    FILE *saved_err;

    /* Core-driven reduction: medium to low on a 55C core average while the
     * EC reading holds at 85C. The first two calls drain the freshly
     * engaged hold; only the third reduces and emits the log line. */
    FILE *saved_stdout = capture_fan_log(log_path);
    enum set_fan_status first = set_fan_level();
    enum set_fan_status second = set_fan_level();
    enum set_fan_status third = set_fan_level();
    char log_line[256];
    read_fan_log(log_path, saved_stdout, log_line, sizeof(log_line));
    CHECK(first == FAN_LEVEL_NOT_SET);
    CHECK(second == FAN_LEVEL_NOT_SET);
    CHECK(third == FAN_LEVEL_SET); /* the transition emits the line */
    CHECK(strstr(log_line, "Core average now 55C, fan set to low") != NULL);

    /* Control-driven (unreadable core average): low to off on a 45C ACPI/EC
     * reading. */
    make_temp("hwmon0", "temp2_input", NULL, 0);
    make_temp("hwmon0", "temp3_input", NULL, 0);
    make_temp("hwmon0", "temp4_input", NULL, 0);
    make_temp("hwmon1", "temp1_input", NULL, 45000);
    saved_stdout = capture_fan_log(log_path);
    first = set_fan_level();
    second = set_fan_level();
    third = set_fan_level();
    read_fan_log(log_path, saved_stdout, log_line, sizeof(log_line));
    CHECK(first == FAN_LEVEL_NOT_SET);
    CHECK(second == FAN_LEVEL_NOT_SET);
    CHECK(third == FAN_LEVEL_SET); /* the transition emits the line */
    CHECK(strstr(log_line, "Temperature now 45C, fan set to off") != NULL);

    /* No-core configuration, simulated by demoting the core readings to
     * OTHER and promoting the nvme reading to EC-kind: with no genuine core
     * average, reductions must follow the control temperature rather than
     * the mean of the EC-kind readings (68C, below their 95C maximum). */
    enum SensorKind core_kind = SENSOR_CPU_CORE;
    sensor_for_path("hwmon0/temp2_input")->kind = SENSOR_OTHER;
    sensor_for_path("hwmon0/temp3_input")->kind = SENSOR_OTHER;
    sensor_for_path("hwmon0/temp4_input")->kind = SENSOR_OTHER;
    sensor_for_path("hwmon2/temp1_input")->kind = SENSOR_CPU;
    size_t saved_core_count = sensor_set.num_core_sensors;
    size_t saved_control_count = sensor_set.num_control_sensors;
    sensor_set.num_core_sensors = 0;
    sensor_set.num_control_sensors = 2;
    make_temp("hwmon1", "temp1_input", NULL, 95000);
    struct FanTemps temps = get_fan_temps();
    CHECK(temps.average_is_core == false);
    CHECK(temps.average_temp == 95); /* folded up, not the 68C EC-kind mean */
    CHECK(temps.control_temp == 95);
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* panic engages maximum */
    CHECK(current_rule == rules + FAN_MAX);
    make_temp("hwmon1", "temp1_input", NULL, 85000);
    for (int i = 0; i < 5; i++) {
        set_fan_level();
        CHECK(current_rule == rules + FAN_MAX); /* reduction follows control */
    }
    sensor_for_path("hwmon0/temp2_input")->kind = core_kind;
    sensor_for_path("hwmon0/temp3_input")->kind = core_kind;
    sensor_for_path("hwmon0/temp4_input")->kind = core_kind;
    sensor_for_path("hwmon2/temp1_input")->kind = SENSOR_OTHER;
    sensor_set.num_core_sensors = saved_core_count;
    sensor_set.num_control_sensors = saved_control_count;

    /* A custom threshold can sit above the panic band: maximum must be
     * held outright while the reading persists, independent of discounted
     * thresholds. */
    char config_file[640];
    expect(snprintf(config_file, sizeof(config_file), "%s/zcfan.conf",
                    fixture_root) > 0);
    write_file(config_file, "max_temp 115\n");
    config_path = config_file;
    get_config();
    CHECK(rules[FAN_MAX].threshold == 115);
    current_rule = rules + FAN_OFF;
    memset(level_ticks, 0, sizeof(level_ticks));
    /* The configured threshold must govern engagement: 94C stays below a
     * 115C maximum for the whole debounce window (with the default 90C it
     * would engage on the tenth tick). */
    make_temp("hwmon1", "temp1_input", NULL, 94000);
    for (int i = 0; i < 10; i++) {
        CHECK(set_fan_level() == FAN_LEVEL_NOT_SET);
        CHECK(current_rule == rules + FAN_OFF);
    }
    /* the band engages and holds independently of the threshold */
    make_temp("hwmon1", "temp1_input", NULL, 95000);
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* panic engages maximum */
    CHECK(current_rule == rules + FAN_MAX);
    for (int i = 0; i < 5; i++) {
        CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* held at the edge */
        CHECK(current_rule == rules + FAN_MAX);
    }
    rules[FAN_MAX].threshold = 90;
    config_path = CONFIG_PATH;

    /* Config safety validation must reject dangerous directives at
     * startup (the child's exit status proves the parse exits). */
    expect_config_status("max_level 0\n", 1);
    expect_config_status("max_level auto\n", 1);
    expect_config_status("temp_hysteresis -1\n", 1);
    expect_config_status("temp_hysteresis 101\n", 1);
    expect_config_status("watchdog_secs 1\n", 1);
    expect_config_status("watchdog_secs 121\n", 1);
    expect_config_status("watchdog_secs 120\nmax_temp 85\nmax_level 7\n"
                         "temp_hysteresis 100\n",
                         0);

    /* The trajectory's last tick sat in the 95C panic band; the runaway
     * module below drives the watchdog path directly (no set_fan_level()
     * in between), so leave the state it reads -- panic/fail-safe flags,
     * held rule, hold ticks -- quiet via one ordinary control tick. */
    make_temp("hwmon1", "temp1_input", NULL, 45000);
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* steps down to off */
    CHECK(current_rule == rules + FAN_OFF);
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* off holds */
    /* The runaway module's first phase exercises failing writes: put the
     * default (unwritable) fan-control file back. */
    fan_control_file = FAN_CONTROL_FILE;

    /* --- Per-fan runaway detection and healing ---
     *
     * The runaway is detected while the off level is held: a fan input
     * above FAN_SELF_RUNNING_RPM for two consecutive refreshes. The
     * excursion writes the maximum level, waits, and re-asserts the held
     * level. By default the fan-control file here is /proc (writes fail),
     * so the failure path is what these tests see first; a fixture file
     * stand-in is swapped in further down for the success-path and
     * exhaustion assertions. The thinkpad hwmon device (hwmon1) gets a
     * fan input for these tests; nothing else reads it (sensor discovery
     * only opens temp*_input files), and the fixture is torn down right
     * after. */

    /* No fan inputs: nothing to detect, no crash, no state churn. */
    current_rule = rules + FAN_OFF;
    reassert_fan_control(); /* must not crash with no fan inputs */
    CHECK(maybe_fix_runaway_fan() == false);

    /* A fan spinning at runaway speed with failing writes: the first
     * refresh only confirms, the second attempts the excursion, which
     * fails and must NOT consume an attempt. The excursion error goes to
     * stderr (uncaptured); the return value stays false every call. */
    make_temp("hwmon1", "fan1_input", NULL, 2500);
    CHECK(maybe_fix_runaway_fan() == false); /* first sight: confirm only */
    for (unsigned int i = 0; i <= FAN_RUNAWAY_RETRY_TICKS * 3; i++) {
        CHECK(maybe_fix_runaway_fan() == false);
    }

    /* Confirmation state is dropped when the held level is not off: an
     * observation before leaving off must not survive the round trip.
     * With failing writes the return value cannot distinguish a
     * confirmation-only refresh from a failed excursion attempt, so the
     * boundary is asserted in the fixture success path further down. */
    current_rule = rules + FAN_LOW;
    CHECK(maybe_fix_runaway_fan() == false); /* no off gate: state dropped */
    current_rule = rules + FAN_OFF;
    CHECK(maybe_fix_runaway_fan() == false); /* must re-confirm */
    CHECK(maybe_fix_runaway_fan() == false); /* excursion attempt, fails */
    for (unsigned int i = 0; i < FAN_RUNAWAY_RETRY_TICKS; i++) {
        CHECK(maybe_fix_runaway_fan() == false); /* drain the backoff */
    }

    /* Success path: point the fan-control file at the fixture file so the
     * excursion write succeeds. The excursion consumes attempt 1, waits
     * out the retry spacing, consumes attempt 2; the third consumes the
     * last attempt, and the refresh after it exhausts the episode: one
     * stderr abandonment line, then holding. */
    fan_control_file = fan_control_fixture;
    saved_stdout = capture_fan_log(log_path);
    CHECK(maybe_fix_runaway_fan() == true); /* excursion 1 */
    read_fan_log(log_path, saved_stdout, log_line, sizeof(log_line));
    CHECK(strstr(log_line,
                 "Fan self-running at 2500 RPM while level off is commanded") !=
          NULL);
    for (unsigned int i = 0; i < FAN_RUNAWAY_RETRY_TICKS; i++) {
        CHECK(maybe_fix_runaway_fan() == false);
    }
    CHECK(maybe_fix_runaway_fan() == true); /* excursion 2 */
    for (unsigned int i = 0; i < FAN_RUNAWAY_RETRY_TICKS; i++) {
        CHECK(maybe_fix_runaway_fan() == false);
    }
    CHECK(maybe_fix_runaway_fan() == true); /* excursion 3: budget spent */
    /* The refresh after the last excursion exhausts the episode: no
     * excursion, one abandonment line on stderr, then holding. */
    saved_err = capture_stderr_log(err_path);
    CHECK(maybe_fix_runaway_fan() == false);
    read_stderr_log(err_path, saved_err, err_line, sizeof(err_line));
    CHECK(strstr(err_line,
                 "Fan still self-running at the maximum level after 3 "
                 "excursion(s)") != NULL);
    for (unsigned int i = 0; i < FAN_RUNAWAY_RETRY_TICKS * 2; i++) {
        CHECK(maybe_fix_runaway_fan() == false); /* holding, silent */
    }

    /* The fan stops: the hold lifts with one stderr re-arm line, and a
     * fresh episode starts from scratch. The gate round trip (off →
     * low → off) must also drop the confirmation: the first call back
     * at off confirms only (false), despite the pre-trip observation. */
    make_temp("hwmon1", "fan1_input", NULL, 0);
    saved_err = capture_stderr_log(err_path);
    CHECK(maybe_fix_runaway_fan() == false);
    read_stderr_log(err_path, saved_err, err_line, sizeof(err_line));
    CHECK(strstr(err_line, "runaway handling re-armed") != NULL);
    make_temp("hwmon1", "fan1_input", NULL, 2500);
    CHECK(maybe_fix_runaway_fan() == false); /* re-armed: confirm again */
    current_rule = rules + FAN_LOW;
    CHECK(maybe_fix_runaway_fan() == false); /* gate drops confirmation */
    current_rule = rules + FAN_OFF;
    CHECK(maybe_fix_runaway_fan() == false); /* confirms again (m4) */
    CHECK(maybe_fix_runaway_fan() == true);  /* fresh excursion 1 */
    /* The excursion is invisible in the file (each write truncates it),
     * but the restore write is not: the held level must be back on the
     * wire after a successful excursion. */
    char content[64];
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level 0") == 0);

    /* The invalid-temperature fail-safe must not be fought by healing or
     * by the held-level reassertions: with every temperature unreadable
     * the daemon commands full-speed, a spinning fan under that command is
     * not a runaway, and rewriting the held off level over the fail-safe
     * would undo it. */
    make_temp("hwmon0", "temp1_input", NULL, 0);
    make_temp("hwmon0", "temp2_input", NULL, 0);
    make_temp("hwmon0", "temp3_input", NULL, 0);
    make_temp("hwmon0", "temp4_input", NULL, 0);
    make_temp("hwmon0", "temp5_input", NULL, 0);
    make_temp("hwmon0", "temp6_input", NULL, 0);
    make_temp("hwmon1", "temp1_input", NULL, 0);
    make_temp("hwmon1", "temp2_input", NULL, 0);
    make_temp("hwmon1", "temp3_input", NULL, 0);
    make_temp("hwmon1", "temp4_input", NULL, 0);
    make_temp("hwmon2", "temp1_input", NULL, 0);
    /* The outage diagnostic is rate-limited: the transition into the
     * outage logs exactly once, a repeat outage tick is silent. */
    saved_err = capture_stderr_log(err_path);
    CHECK(set_fan_level() == FAN_LEVEL_INVALID); /* fail-safe engaged */
    read_stderr_log(err_path, saved_err, err_line, sizeof(err_line));
    CHECK(strstr(err_line, "Couldn't find any valid temperature") != NULL);
    saved_err = capture_stderr_log(err_path);
    CHECK(set_fan_level() == FAN_LEVEL_INVALID); /* repeat tick */
    expect_stderr_empty(err_path, saved_err);    /* rate-limited */
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level full-speed") == 0);
    /* A watchdog refresh under the fail-safe: the status reassertion (the
     * EC reads "enabled") must re-write the fail-safe level, not the held
     * off level, and healing must stay suspended and silent. */
    write_file(fan_control_fixture, "status: enabled\n");
    saved_err = capture_stderr_log(err_path);
    reassert_fan_control();
    expect_stderr_empty(err_path, saved_err);
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level full-speed") == 0);
    /* Temps recover while the fan-control file is unwritable: the restore
     * fails, the fail-safe must stay active (the next tick retries instead
     * of trusting bookkeeping that no longer matches the wire), and once
     * the file is writable again the held level is re-written. */
    make_temp("hwmon0", "temp2_input", NULL, 52000);
    make_temp("hwmon0", "temp3_input", NULL, 55000);
    make_temp("hwmon0", "temp4_input", NULL, 58000);
    make_temp("hwmon1", "temp1_input", NULL, 45000);
    fan_control_file = "/nonexistent-zcfan-fan-control";
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* restore failed */
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level full-speed") == 0); /* wire untouched */
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET);     /* retried, failed again */
    fan_control_file = fan_control_fixture;
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* off held again */
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level 0") == 0);  /* held level re-written */
    CHECK(maybe_fix_runaway_fan() == false); /* fresh confirmation 1 */
    CHECK(maybe_fix_runaway_fan() == true);  /* fresh excursion 1 */
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level 0") == 0);

    /* Recovery in the panic band must not command the held off level: the
     * recovered temperature already demands the maximum, so the panic
     * write takes the wire directly. */
    make_temp("hwmon0", "temp2_input", NULL, 0);
    make_temp("hwmon0", "temp3_input", NULL, 0);
    make_temp("hwmon0", "temp4_input", NULL, 0);
    make_temp("hwmon1", "temp1_input", NULL, 0);
    make_temp("hwmon2", "temp1_input", NULL, 0);
    /* Recovery reset the outage diagnostic: the new outage logs again. */
    saved_err = capture_stderr_log(err_path);
    CHECK(set_fan_level() == FAN_LEVEL_INVALID); /* fail-safe re-engaged */
    read_stderr_log(err_path, saved_err, err_line, sizeof(err_line));
    CHECK(strstr(err_line, "Couldn't find any valid temperature") != NULL);
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level full-speed") == 0);
    make_temp("hwmon0", "temp2_input", NULL, 45000);
    make_temp("hwmon0", "temp3_input", NULL, 47000);
    make_temp("hwmon0", "temp4_input", NULL, 49000);
    make_temp("hwmon1", "temp1_input", NULL, 96000);
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* panic, no off in between */
    CHECK(current_rule == rules + FAN_MAX);
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level full-speed") == 0);
    /* Healing stays suspended in the panic band even when a failed panic
     * write leaves the bookkeeping at off: no excursion, silent. */
    fan_control_file = "/nonexistent-zcfan-fan-control";
    current_rule = rules + FAN_OFF;
    CHECK(set_fan_level() == FAN_LEVEL_NOT_SET); /* panic write failed */
    CHECK(current_rule == rules + FAN_OFF);
    saved_err = capture_stderr_log(err_path);
    CHECK(maybe_fix_runaway_fan() == false); /* panic gate: suspended */
    CHECK(maybe_fix_runaway_fan() == false);
    expect_stderr_empty(err_path, saved_err);
    fan_control_file = fan_control_fixture;
    CHECK(set_fan_level() == FAN_LEVEL_SET); /* panic retried */
    CHECK(current_rule == rules + FAN_MAX);
    /* Settle back to off for the fan-at-rest test that follows. */
    make_temp("hwmon1", "temp1_input", NULL, 45000);
    {
        const struct Rule *settle_path[] = {
            rules + FAN_MAX, rules + FAN_MAX, rules + FAN_OFF,
            rules + FAN_OFF, rules + FAN_OFF,
        };
        for (int i = 0; i < 5; i++) {
            const struct Rule *before = current_rule;
            enum set_fan_status st = set_fan_level();
            CHECK(current_rule == settle_path[i]);
            CHECK(st ==
                  (current_rule != before ? FAN_LEVEL_SET : FAN_LEVEL_NOT_SET));
        }
    }
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level 0") == 0);

    /* A fan at rest never triggers anything. */
    make_temp("hwmon1", "fan1_input", NULL, 0);
    CHECK(maybe_fix_runaway_fan() == false);
    CHECK(maybe_fix_runaway_fan() == false);

    /* An out-of-range RPM reading must degrade to unreadable: the gate
     * sees a stopped fan and resets, silently, instead of attempting an
     * excursion on garbage. (Placed after the at-rest reset so it does
     * not clobber the confirmation state the success path relied on.) */
    expect(snprintf(path, sizeof(path), "%s/hwmon1/fan1_input", fixture_root) >
           0);
    write_file(path, "99999999999999999999");
    saved_err = capture_stderr_log(err_path);
    CHECK(maybe_fix_runaway_fan() == false);
    CHECK(maybe_fix_runaway_fan() == false);
    expect_stderr_empty(err_path, saved_err);
    make_temp("hwmon1", "fan1_input", NULL, 2500);

    /* A suspend signal interrupting the excursion pause must not skip the
     * restore, and must not spend the full second: the interrupted pause
     * breaks early once pending_sleep is observed. SIGALRM reuses the
     * SIGPWR handler (both just set pending_sleep); the timer is armed
     * after the confirmation-only refresh so it fires inside the sleep. */
    make_temp("hwmon1", "fan1_input", NULL, 2500);
    CHECK(maybe_fix_runaway_fan() == false); /* confirm only */
    const struct sigaction pausing = {.sa_handler = handle_sigpwr};
    const struct sigaction sigalrm_old = {.sa_handler = SIG_DFL};
    expect(sigaction(SIGALRM, &pausing, NULL) == 0);
    struct itimerval timer = {0};
    timer.it_value.tv_usec = 50000; /* interrupt the pause at 50ms */
    expect(setitimer(ITIMER_REAL, &timer, NULL) == 0);
    struct timespec pause_start, pause_end;
    expect(clock_gettime(CLOCK_MONOTONIC, &pause_start) == 0);
    CHECK(maybe_fix_runaway_fan() == true); /* excursion, pause broken */
    expect(clock_gettime(CLOCK_MONOTONIC, &pause_end) == 0);
    CHECK(pause_end.tv_sec - pause_start.tv_sec < 1); /* not a full second */
    expect(sigaction(SIGALRM, &sigalrm_old, NULL) == 0);
    pending_sleep = 0;
    read_text_file(fan_control_fixture, content, sizeof(content));
    CHECK(strcmp(content, "level 0") == 0); /* restored despite the signal */

    fan_control_file = FAN_CONTROL_FILE;
    close_sensor_fds(&sensor_set);
    rm_rf(fixture_root);

    if (num_failures == 0) {
        printf("All tests passed.\n");
        return 0;
    }
    printf("%d test(s) failed.\n", num_failures);
    return 1;
}
