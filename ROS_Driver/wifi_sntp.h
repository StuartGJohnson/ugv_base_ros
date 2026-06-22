#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include <esp_sntp.h>
#include <esp_timer.h>

static volatile bool     g_time_sync_seen = false;
static volatile uint32_t g_time_sync_count = 0;
static volatile int64_t  g_last_sync_mono_us = 0;

// Called by SNTP when a sync occurs.
void on_sntp_time_sync(struct timeval *tv)
{
  g_time_sync_seen = true;
  g_time_sync_count++;
  g_last_sync_mono_us = esp_timer_get_time();
}

void setup_sntp_time(const char* ntp_server)
{
  // Use smooth clock adjustment for small corrections.
  sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);

  // 5 minutes. Units are milliseconds.
  // Do not make this silly-short; SNTP is not meant to be polled at high rate.
  //sntp_set_sync_interval(5UL * 60UL * 1000UL);
  // temporary testing
  sntp_set_sync_interval(1UL * 60UL * 1000UL);

  // Lets telemetry know when real SNTP syncs occurred.
  sntp_set_time_sync_notification_cb(on_sntp_time_sync);

  // UTC. No timezone offset. This starts SNTP internally.
  configTime(0, 0, ntp_server);
}

int64_t sntp_time_us()
{
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec;
}

// monotonic (mcu local) time
int64_t mono_time_us()
{
  return esp_timer_get_time();
}

bool sntp_time_valid()
{
  // Reject 1970-ish boot time. Pick a conservative modern threshold.
  time_t now = time(nullptr);
  return g_time_sync_seen && now > 1700000000;  // ~2023-11-14 UTC
}

int64_t average_time_us(int64_t t0_us, int64_t t1_us)
{
  // Avoid overflow, even though these values are not near int64 overflow.
  return t0_us + (t1_us - t0_us) / 2;
}

String sntp_time_to_string(int64_t wall_us)
{
    time_t sec = wall_us / 1000000LL;
    int32_t usec = wall_us % 1000000LL;

    struct tm tm_utc;
    gmtime_r(&sec, &tm_utc);

    char buf[40];

    snprintf(buf, sizeof(buf),
             "%04d-%02d-%02dT%02d:%02d:%02d.%06ldZ",
             tm_utc.tm_year + 1900,
             tm_utc.tm_mon + 1,
             tm_utc.tm_mday,
             tm_utc.tm_hour,
             tm_utc.tm_min,
             tm_utc.tm_sec,
             (long)usec);

    return String(buf);
}

// update oled with current time
void updateOledTimingInfo() {
  bool t_valid = sntp_time_valid();
  int max_delay = 10000;
  int accum_delay = 0;
  while (!t_valid & accum_delay<max_delay){
    t_valid = sntp_time_valid();
    delay(1000);
    accum_delay = accum_delay + 1000;
  }

  int64_t t = sntp_time_us();
  String timestamp = sntp_time_to_string(t);
  screenLine_0 = String("T:") + timestamp;
  if (t_valid) {
    screenLine_1 = String("Time Valid");
  }
  else {
    screenLine_1 = String("Time Not Valid");
  }
  oled_update();
}

void updateOledTimingInfoShort() {
  screenLine_2 = "sntp_updates:" + String(g_time_sync_count);
}

