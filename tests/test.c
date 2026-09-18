/* Functional tests for the sensor discovery and refresh logic in zcfan.c.
 *
 * The daemon source is compiled in directly (with main renamed), and the
 * hwmon sysfs tree is faked with regular files in a temporary directory, so
 * the tests need no privileges and no real hardware. */

#define main zcfan_unused_main
#include "../zcfan.c"
#undef main

#include <sys/stat.h>

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

static const struct Sensor *sensor_for_path(const char *path) {
    for (size_t i = 0; i < sensor_set.num_sensor_fds; i++) {
        if (strcmp(sensor_set.sensors[i].path, path) == 0)
            return &sensor_set.sensors[i];
    }
    return NULL;
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
    CHECK(sensor_set.num_cpu_core_sensors == 3);
    CHECK(sensor_set.num_cpu_temp_sensors == 4);
    CHECK(sensor_set.num_ignored_sensors == 0);
    CHECK(sensor_for_path("hwmon0/temp2_input")->kind == SENSOR_CPU_CORE);
    CHECK(sensor_for_path("hwmon0/temp1_input")->kind ==
          SENSOR_OTHER); /* die/package reading: excluded from control */
    CHECK(sensor_for_path("hwmon1/temp1_input")->kind == SENSOR_CPU);
    CHECK(sensor_for_path("hwmon2/temp1_input")->kind == SENSOR_OTHER);
    for (size_t i = 1; i < sensor_set.num_sensor_fds; i++) {
        CHECK(strcmp(sensor_set.sensors[i - 1].path,
                     sensor_set.sensors[i].path) < 0);
    }
    CHECK(get_average_temp() == 52); /* mean of the three cores */

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
    CHECK(sensor_set.num_cpu_core_sensors == 4);
    CHECK(get_average_temp() == 54);
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
    CHECK(sensor_set.num_cpu_core_sensors == 3);
    CHECK(get_average_temp() == 52);

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
    CHECK(sensor_set.num_cpu_core_sensors == 3);
    CHECK(sensor_for_path("hwmon0/temp2_input")->ino != ino_before);
    CHECK(get_average_temp() == 68); /* 99000 replaces 50000 in the mean */

    /* An unreadable input makes the scan incomplete: the partial snapshot
     * must be discarded and the active set retained. */
    expect(snprintf(path, sizeof(path), "%s/hwmon2/temp9_input", fixture_root) >
           0);
    expect(symlink("/nonexistent-target", path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(get_average_temp() == 68);
    expect(unlink(path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(get_average_temp() == 68);

    /* A hwmon device that vanishes mid-scan (here: dangling symlink): its
     * directory open fails, the scan is incomplete, and the active set must
     * be retained. */
    expect(snprintf(path, sizeof(path), "%s/hwmon9", fixture_root) > 0);
    expect(symlink("/nonexistent-target", path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(get_average_temp() == 68);
    expect(unlink(path) == 0);
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(get_average_temp() == 68);

    /* A failing hwmon root must not crash the daemon (this exits on the
     * first tick; first_tick is cleared here to model steady state). */
    first_tick = 0;
    hwmon_root = "/nonexistent-zcfan-test";
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(get_average_temp() == 68);
    hwmon_root = fixture_root;
    refresh_sensors();
    CHECK(sensor_set.num_sensor_fds == 7);
    CHECK(get_average_temp() == 68);

    /* The effective temperature is the maximum of the core average and the
     * ACPI/EC sensor reading: that sensor is what the firmware's critical
     * shutdown trip reacts to. */
    expect(snprintf(path, sizeof(path), "%s/hwmon1/temp1_input", fixture_root) >
           0);
    write_file(path, "95000");
    CHECK(get_average_temp() == 95); /* ACPI/EC sensor wins over the average */
    write_file(path, "60000");
    CHECK(get_average_temp() == 68); /* 68C average beats 60C ACPI/EC */

    /* A die/package spike must not move the control temperature: hwmon0's
     * "Package id 0" is deliberately excluded from fan control. */
    expect(snprintf(path, sizeof(path), "%s/hwmon0/temp1_input", fixture_root) >
           0);
    write_file(path, "99000");
    CHECK(get_average_temp() == 68); /* die reading ignored */

    /* Every hwmon0 reading invalid (0) leaves num_valid_temps == 0 while
     * cores are still selected: the ACPI/EC sensor (60000) must be
     * returned via the cpu_max path instead of erroring. */
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
    CHECK(get_average_temp() == 60); /* hwmon1's ACPI/EC sensor survives */

    /* The 95C panic path engages maximum immediately, bypassing debounce:
     * from FAN_OFF with no debounce ticks accrued, the level still moves.
     * (The fan write itself fails without thinkpad_acpi; ignored.) */
    expect(snprintf(path, sizeof(path), "%s/hwmon1/temp1_input", fixture_root) >
           0);
    write_file(path, "95000");
    current_rule = rules + FAN_OFF;
    CHECK(set_fan_level() == FAN_LEVEL_SET);
    CHECK(current_rule == rules + FAN_MAX);

    close_sensor_fds(&sensor_set);
    rm_rf(fixture_root);

    if (num_failures == 0) {
        printf("All tests passed.\n");
        return 0;
    }
    printf("%d test(s) failed.\n", num_failures);
    return 1;
}
