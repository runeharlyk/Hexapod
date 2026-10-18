// Host tests of the metrics page's task table: each task's CPU share over the window between two snapshots, kept
// apart by task number as tasks start and end, across the wrap of the 32-bit run-time counters, and the order that
// puts the stack closest to an overflow first.

#include <unity.h>

#include <cstring>
#include <task_table.h>

using task_table::CpuWindow;
using task_table::Task;

namespace {

Task task(uint32_t number, const char* name, uint32_t runTime, uint32_t stackFreeMin = 1000) {
    Task t {};
    t.number = number;
    std::strncpy(t.name, name, sizeof(t.name) - 1);
    t.runTime = runTime;
    t.stackFreeMin = stackFreeMin;
    return t;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_the_first_snapshot_has_no_window_and_no_cpu() {
    CpuWindow<8> window;
    Task tasks[] = {task(1, "control", 5'000'000), task(2, "IDLE0", 9'000'000)};
    TEST_ASSERT_EQUAL_UINT32(0, window.measure(tasks, 2, 10'000'000));
    TEST_ASSERT_EQUAL_FLOAT(0.f, tasks[0].cpuPercent);
    TEST_ASSERT_EQUAL_FLOAT(0.f, tasks[1].cpuPercent);
}

void test_a_task_shares_one_core_by_its_run_time_over_the_window() {
    CpuWindow<8> window;
    Task first[] = {task(1, "control", 1'000'000), task(2, "IDLE0", 4'000'000)};
    window.measure(first, 2, 10'000'000);
    Task second[] = {task(1, "control", 1'500'000), task(2, "IDLE0", 5'500'000)};
    TEST_ASSERT_EQUAL_UINT32(2'000'000, window.measure(second, 2, 12'000'000));
    TEST_ASSERT_EQUAL_FLOAT(25.f, second[0].cpuPercent);
    TEST_ASSERT_EQUAL_FLOAT(75.f, second[1].cpuPercent);
}

// A snapshot lists the tasks in whatever order the scheduler's lists hold them, which changes as they block and wake.
void test_tasks_are_matched_by_number_not_by_their_place_in_the_snapshot() {
    CpuWindow<8> window;
    Task first[] = {task(1, "control", 1'000'000), task(2, "service", 3'000'000)};
    window.measure(first, 2, 10'000'000);
    Task second[] = {task(2, "service", 3'100'000), task(1, "control", 1'900'000)};
    window.measure(second, 2, 11'000'000);
    TEST_ASSERT_EQUAL_FLOAT(10.f, second[0].cpuPercent);
    TEST_ASSERT_EQUAL_FLOAT(90.f, second[1].cpuPercent);
}

// The previous snapshot saw every task, so one it did not see started within the window and its whole counter is
// the window's; the task that ended takes nothing with it.
void test_a_task_started_in_the_window_counts_its_run_time_since_it_started() {
    CpuWindow<8> window;
    Task first[] = {task(1, "control", 1'000'000), task(7, "ota_download", 2'000'000)};
    window.measure(first, 2, 10'000'000);
    Task second[] = {task(1, "control", 1'200'000), task(9, "camstream", 300'000)};
    window.measure(second, 2, 11'000'000);
    TEST_ASSERT_EQUAL_FLOAT(20.f, second[0].cpuPercent);
    TEST_ASSERT_EQUAL_FLOAT(30.f, second[1].cpuPercent);
}

// FreeRTOS hands a deleted task's number to no other, though a new task can take its handle's memory.
void test_a_new_task_on_a_reused_name_is_not_measured_against_the_old_one() {
    CpuWindow<8> window;
    Task first[] = {task(4, "camstream", 3'000'000)};
    window.measure(first, 1, 10'000'000);
    Task second[] = {task(5, "camstream", 100'000)};
    window.measure(second, 1, 11'000'000);
    TEST_ASSERT_EQUAL_FLOAT(10.f, second[0].cpuPercent);
}

// The counters are 32-bit microseconds on the esp_timer clock, so a busy task's counter wraps within the 71.6 minutes
// of uptime that wrap the clock's own low word.
void test_a_counter_that_wrapped_within_the_window_still_counts_what_it_ran() {
    CpuWindow<8> window;
    Task first[] = {task(1, "IDLE1", 0xFFFF'FF00u)};
    window.measure(first, 1, 0xFFFF'F000ull);
    Task second[] = {task(1, "IDLE1", 0x0000'0100u)};
    TEST_ASSERT_EQUAL_UINT32(0x2000, window.measure(second, 1, 0x1'0000'1000ull));
    TEST_ASSERT_EQUAL_FLOAT(100.f * 0x200 / 0x2000, second[0].cpuPercent);
}

// Over 2^32 us between two requests, a difference modulo 2^32 no longer says how long a task ran.
void test_a_window_the_counters_cannot_span_has_no_cpu_and_the_next_one_does() {
    CpuWindow<8> window;
    Task first[] = {task(1, "control", 1'000'000)};
    window.measure(first, 1, 10'000'000);
    Task second[] = {task(1, "control", 1'500'000)};
    TEST_ASSERT_EQUAL_UINT32(0, window.measure(second, 1, 10'000'000 + (1ull << 32)));
    TEST_ASSERT_EQUAL_FLOAT(0.f, second[0].cpuPercent);
    Task third[] = {task(1, "control", 1'600'000)};
    TEST_ASSERT_EQUAL_UINT32(1'000'000, window.measure(third, 1, 11'000'000 + (1ull << 32)));
    TEST_ASSERT_EQUAL_FLOAT(10.f, third[0].cpuPercent);
}

// A running task's counter is brought up to date only when it is switched out, so one window can be credited with
// time the window before it ran.
void test_a_share_above_one_core_is_held_to_one_core() {
    CpuWindow<8> window;
    Task first[] = {task(1, "IDLE0", 0)};
    window.measure(first, 1, 10'000'000);
    Task second[] = {task(1, "IDLE0", 1'100'000)};
    window.measure(second, 1, 11'000'000);
    TEST_ASSERT_EQUAL_FLOAT(100.f, second[0].cpuPercent);
}

// With more tasks than the baseline holds, a task left out would next be taken for a new one and credited with its
// whole run time; the window starts again instead.
void test_a_snapshot_larger_than_the_baseline_starts_the_window_again() {
    CpuWindow<2> window;
    Task three[] = {task(1, "a", 100), task(2, "b", 200), task(3, "c", 300)};
    window.measure(three, 3, 10'000'000);
    Task two[] = {task(1, "a", 200), task(3, "c", 400)};
    TEST_ASSERT_EQUAL_UINT32(0, window.measure(two, 2, 11'000'000));
    Task again[] = {task(1, "a", 300), task(3, "c", 600)};
    TEST_ASSERT_EQUAL_UINT32(1'000'000, window.measure(again, 2, 12'000'000));
    TEST_ASSERT_EQUAL_FLOAT(0.02f, again[1].cpuPercent);
}

void test_the_tightest_stacks_lead_and_the_roomiest_are_cut() {
    Task tasks[] = {task(1, "service", 0, 2400), task(2, "serial_rx", 0, 5800), task(3, "control", 0, 900),
                    task(4, "IDLE0", 0, 600)};
    TEST_ASSERT_EQUAL_size_t(3, task_table::tightestFirst(tasks, 4, 3));
    TEST_ASSERT_EQUAL_STRING("IDLE0", tasks[0].name);
    TEST_ASSERT_EQUAL_STRING("control", tasks[1].name);
    TEST_ASSERT_EQUAL_STRING("service", tasks[2].name);
}

// The EventBus workers all carry one name, so a tie on name falls to the task number and two replies agree.
void test_ties_order_by_name_then_by_number() {
    Task tasks[] = {task(12, "evtbus", 0, 1800), task(10, "evtbus", 0, 1800), task(3, "dns_server", 0, 1800)};
    TEST_ASSERT_EQUAL_size_t(3, task_table::tightestFirst(tasks, 3, 8));
    TEST_ASSERT_EQUAL_STRING("dns_server", tasks[0].name);
    TEST_ASSERT_EQUAL_UINT32(10, tasks[1].number);
    TEST_ASSERT_EQUAL_UINT32(12, tasks[2].number);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_first_snapshot_has_no_window_and_no_cpu);
    RUN_TEST(test_a_task_shares_one_core_by_its_run_time_over_the_window);
    RUN_TEST(test_tasks_are_matched_by_number_not_by_their_place_in_the_snapshot);
    RUN_TEST(test_a_task_started_in_the_window_counts_its_run_time_since_it_started);
    RUN_TEST(test_a_new_task_on_a_reused_name_is_not_measured_against_the_old_one);
    RUN_TEST(test_a_counter_that_wrapped_within_the_window_still_counts_what_it_ran);
    RUN_TEST(test_a_window_the_counters_cannot_span_has_no_cpu_and_the_next_one_does);
    RUN_TEST(test_a_share_above_one_core_is_held_to_one_core);
    RUN_TEST(test_a_snapshot_larger_than_the_baseline_starts_the_window_again);
    RUN_TEST(test_the_tightest_stacks_lead_and_the_roomiest_are_cut);
    RUN_TEST(test_ties_order_by_name_then_by_number);
    return UNITY_END();
}
