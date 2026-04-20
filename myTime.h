#pragma once
#include <time.h>

void set_system_time_unix(time_t t) {
  struct timeval tv = {.tv_sec = t, .tv_usec = 0};
  settimeofday(&tv, nullptr);
}

static void set_system_time_ymdhms(int y, int mon, int d, int h, int min, int s) {
  struct tm tmv = {};
  tmv.tm_year = y - 1900;
  tmv.tm_mon = mon - 1;
  tmv.tm_mday = d;
  tmv.tm_hour = h;
  tmv.tm_min = min;
  tmv.tm_sec = s;
  time_t t = mktime(&tmv);
  if (t > 0) set_system_time_unix(t);
}

static int64_t now_rtc_s() {
  timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec;
}


static bool get_current_tm(struct tm &out) {
  time_t now_s = (time_t)now_rtc_s();
  if (now_s <= 0) return false;
  gmtime_r(&now_s, &out);
  return true;
}

static int current_hour() {
  struct tm tmv;
  if (!get_current_tm(tmv)) return 0;
  return tmv.tm_hour;
}

static int current_minute() {
  struct tm tmv;
  if (!get_current_tm(tmv)) return 0;
  return tmv.tm_min;
}

static int current_second() {
  struct tm tmv;
  if (!get_current_tm(tmv)) return 0;
  return tmv.tm_sec;
}


String getFormattedTimeLibString() {
  struct tm tmv;
  if (!get_current_tm(tmv)) return "1970-01-01 00:00:00";
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return String(buf);
}

String getFormattedUnixTime(uint32_t unix_time) {
    time_t t = (time_t)unix_time;
    struct tm tmv;
    gmtime_r(&t, &tmv);

    char buf[24];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d_%02d:%02d:%02d", // we print with _ because this function is used in post body
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);

    return String(buf);
}