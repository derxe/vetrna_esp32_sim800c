constexpr int monthFromStr(const char* m) {
  return
    (m[0]=='J'&&m[1]=='a') ? 1 :
    (m[0]=='F')           ? 2 :
    (m[0]=='M'&&m[2]=='r')? 3 :
    (m[0]=='A'&&m[1]=='p')? 4 :
    (m[0]=='M'&&m[2]=='y')? 5 :
    (m[0]=='J'&&m[2]=='n')? 6 :
    (m[0]=='J'&&m[2]=='l')? 7 :
    (m[0]=='A'&&m[1]=='u')? 8 :
    (m[0]=='S')           ? 9 :
    (m[0]=='O')           ? 10:
    (m[0]=='N')           ? 11:
                             12;
}

constexpr bool isLeap(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

constexpr int daysBeforeMonth(int y, int m) {
  constexpr int d[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  return d[m-1] + (m > 2 && isLeap(y));
}

constexpr int buildUnixTime() {
  const char* d = __DATE__; // "Mmm dd yyyy"
  const char* t = __TIME__; // "hh:mm:ss"

  int year  = (d[7]-'0')*1000 + (d[8]-'0')*100 +
              (d[9]-'0')*10   + (d[10]-'0');
  int month = monthFromStr(d);
  int day   = (d[4]==' ' ? d[5]-'0' : (d[4]-'0')*10 + d[5]-'0');

  int hour = (t[0]-'0')*10 + t[1]-'0';
  int min  = (t[3]-'0')*10 + t[4]-'0';
  int sec  = (t[6]-'0')*10 + t[7]-'0';

  int days = 0;
  for (int y = 1970; y < year; ++y)
    days += isLeap(y) ? 366 : 365;

  days += daysBeforeMonth(year, month) + (day - 1);

  return days * 86400 + hour * 3600 + min * 60 + sec;
}

constexpr uint32_t BUILD_UNIX_TIME = buildUnixTime();

