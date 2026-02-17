#include <Wire.h>
#include <RTClib.h>
#include <avr/pgmspace.h>

// =========================
// Configuration
// =========================
#define SET_RTC_ON_UPLOAD 0      // 1 = set RTC to compile time once on upload
#define ENABLE_SERIAL_DEBUG 1    // 1 = print status once per minute
#define USE_DEBUG_LED 1          // 1 = mirror state to onboard LED (D13)

static const uint8_t LED_PIN = 3;
static const uint8_t DEBUG_LED_PIN = 13;

static const unsigned long RTC_POLL_MS = 200UL;
static const unsigned long PREWARN_BLINK_INTERVAL_MS = 500UL; // 1 Hz (500 ms on / 500 ms off)
static const unsigned long ERROR_BLINK_INTERVAL_MS = 100UL;   // 5 Hz (100 ms on / 100 ms off)
static const unsigned long ERROR_SERIAL_INTERVAL_MS = 5000UL;
static const int PREWARN_SECONDS = 30;
static const size_t SERIAL_CMD_BUF_LEN = 40;

// =========================
// RTC
// =========================
RTC_DS3231 rtc;
bool rtcAvailable = false;

// Important design choice:
// The DS3231 is kept in local Germany time (Europe/Berlin), not UTC.
// DST is evaluated from this local time using EU rules (last Sunday Mar/Oct).

// =========================
// Holidays NRW 2026..2035
// =========================
static const uint16_t HOLIDAY_START_YEAR = 2026;
static const uint16_t HOLIDAY_END_YEAR = 2035;
static const uint8_t HOLIDAYS_PER_YEAR = 11;

const uint32_t NRW_HOLIDAYS_2026_2035[] PROGMEM = {
  // 2026
  20260101UL, 20260403UL, 20260406UL, 20260501UL, 20260514UL, 20260525UL, 20260604UL, 20261003UL, 20261101UL, 20261225UL, 20261226UL,
  // 2027
  20270101UL, 20270326UL, 20270329UL, 20270501UL, 20270506UL, 20270517UL, 20270527UL, 20271003UL, 20271101UL, 20271225UL, 20271226UL,
  // 2028
  20280101UL, 20280414UL, 20280417UL, 20280501UL, 20280525UL, 20280605UL, 20280615UL, 20281003UL, 20281101UL, 20281225UL, 20281226UL,
  // 2029
  20290101UL, 20290330UL, 20290402UL, 20290501UL, 20290510UL, 20290521UL, 20290531UL, 20291003UL, 20291101UL, 20291225UL, 20291226UL,
  // 2030
  20300101UL, 20300419UL, 20300422UL, 20300501UL, 20300530UL, 20300610UL, 20300620UL, 20301003UL, 20301101UL, 20301225UL, 20301226UL,
  // 2031
  20310101UL, 20310411UL, 20310414UL, 20310501UL, 20310522UL, 20310602UL, 20310612UL, 20311003UL, 20311101UL, 20311225UL, 20311226UL,
  // 2032
  20320101UL, 20320326UL, 20320329UL, 20320501UL, 20320506UL, 20320517UL, 20320527UL, 20321003UL, 20321101UL, 20321225UL, 20321226UL,
  // 2033
  20330101UL, 20330415UL, 20330418UL, 20330501UL, 20330526UL, 20330606UL, 20330616UL, 20331003UL, 20331101UL, 20331225UL, 20331226UL,
  // 2034
  20340101UL, 20340407UL, 20340410UL, 20340501UL, 20340518UL, 20340529UL, 20340608UL, 20341003UL, 20341101UL, 20341225UL, 20341226UL,
  // 2035
  20350101UL, 20350323UL, 20350326UL, 20350501UL, 20350503UL, 20350514UL, 20350524UL, 20351003UL, 20351101UL, 20351225UL, 20351226UL
};

// =========================
// Time interval model
// Intervals are [start, end) in seconds since midnight.
// =========================
struct Interval {
  int32_t startSec;
  int32_t endSec;
};

enum DayType : uint8_t {
  DAYTYPE_WEEKDAY = 0,
  DAYTYPE_WEEKEND_OR_HOLIDAY = 1
};

enum LedState : uint8_t {
  LED_OFF_ALLOWED = 0,
  LED_BLINK_PREWARN = 1,
  LED_ON_FORBIDDEN = 2,
  LED_ERROR_RTC_INVALID = 3
};

static const Interval SUMMER_WEEKDAY_INTERVALS[] = {
  {5L * 3600L + 30L * 60L, 13L * 3600L},
  {15L * 3600L, 23L * 3600L}
};
static const Interval SUMMER_WEEKEND_HOL_INTERVALS[] = {
  {7L * 3600L, 13L * 3600L},
  {15L * 3600L, 23L * 3600L}
};
static const Interval WINTER_WEEKDAY_INTERVALS[] = {
  {5L * 3600L + 30L * 60L, 22L * 3600L}
};
static const Interval WINTER_WEEKEND_HOL_INTERVALS[] = {
  {7L * 3600L, 13L * 3600L},
  {15L * 3600L, 23L * 3600L}
};

LedState currentState = LED_OFF_ALLOWED;
unsigned long lastRtcPollMs = 0;
uint8_t lastDebugSecond = 255;
bool rtcInvalidLatched = false;
unsigned long lastErrorSerialMs = 0;
char serialCmdBuf[SERIAL_CMD_BUF_LEN];
size_t serialCmdLen = 0;

// =========================
// Utility functions
// =========================
int32_t secondsSinceMidnight(uint8_t h, uint8_t m, uint8_t s) {
  return (int32_t)h * 3600L + (int32_t)m * 60L + (int32_t)s;
}

bool isLeapYear(uint16_t year) {
  return ((year % 4U == 0U) && (year % 100U != 0U)) || (year % 400U == 0U);
}

uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t dim[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 0;
  if (month == 2 && isLeapYear(year)) return 29;
  return dim[month - 1];
}

bool isValidDateTimeFields(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  if (year < 2000 || year > 2099) return false;
  if (month < 1 || month > 12) return false;
  const uint8_t maxDay = daysInMonth(year, month);
  if (day < 1 || day > maxDay) return false;
  if (hour > 23) return false;
  if (minute > 59) return false;
  if (second > 59) return false;
  return true;
}

bool parse2Digits(const char *p, uint8_t &out) {
  if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return false;
  out = (uint8_t)((p[0] - '0') * 10 + (p[1] - '0'));
  return true;
}

bool parse4Digits(const char *p, uint16_t &out) {
  for (uint8_t i = 0; i < 4; ++i) {
    if (p[i] < '0' || p[i] > '9') return false;
  }
  out = (uint16_t)((p[0] - '0') * 1000 + (p[1] - '0') * 100 + (p[2] - '0') * 10 + (p[3] - '0'));
  return true;
}

void printSerialCommandHelp() {
#if ENABLE_SERIAL_DEBUG
  Serial.println(F("Serial command: SET YYYY-MM-DD HH:MM:SS"));
#endif
}

void handleSerialLine(char *line) {
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == '\n')) {
    line[n - 1] = '\0';
    --n;
  }

  if (strncmp(line, "SET ", 4) != 0) {
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: Unknown command"));
    printSerialCommandHelp();
#endif
    return;
  }

  const char *p = line + 4;
  // Expected exact format: YYYY-MM-DD HH:MM:SS (19 chars after "SET ")
  if (strlen(p) != 19 || p[4] != '-' || p[7] != '-' || p[10] != ' ' || p[13] != ':' || p[16] != ':') {
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: Bad SET format"));
    printSerialCommandHelp();
#endif
    return;
  }

  uint16_t year = 0;
  uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0;
  if (!parse4Digits(&p[0], year) ||
      !parse2Digits(&p[5], month) ||
      !parse2Digits(&p[8], day) ||
      !parse2Digits(&p[11], hour) ||
      !parse2Digits(&p[14], minute) ||
      !parse2Digits(&p[17], second)) {
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: SET contains invalid digits"));
#endif
    return;
  }

  if (!isValidDateTimeFields(year, month, day, hour, minute, second)) {
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: SET datetime out of range"));
#endif
    return;
  }

  if (!rtcAvailable) {
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: RTC not available"));
#endif
    return;
  }

  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  rtcInvalidLatched = false;
  currentState = LED_OFF_ALLOWED;
  lastDebugSecond = 255; // force immediate status print

#if ENABLE_SERIAL_DEBUG
  Serial.print(F("OK: RTC set to "));
  Serial.print(year);
  Serial.print('-');
  if (month < 10) Serial.print('0');
  Serial.print(month);
  Serial.print('-');
  if (day < 10) Serial.print('0');
  Serial.print(day);
  Serial.print(' ');
  if (hour < 10) Serial.print('0');
  Serial.print(hour);
  Serial.print(':');
  if (minute < 10) Serial.print('0');
  Serial.print(minute);
  Serial.print(':');
  if (second < 10) Serial.print('0');
  Serial.println(second);
#endif
}

void processSerialCommands() {
#if ENABLE_SERIAL_DEBUG
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        handleSerialLine(serialCmdBuf);
        serialCmdLen = 0;
      }
      continue;
    }

    if (serialCmdLen < (SERIAL_CMD_BUF_LEN - 1)) {
      serialCmdBuf[serialCmdLen++] = c;
    } else {
      serialCmdLen = 0;
      Serial.println(F("ERROR: command too long"));
    }
  }
#endif
}

bool isHolidayNRW(uint16_t year, uint8_t month, uint8_t day) {
  if (year < HOLIDAY_START_YEAR || year > HOLIDAY_END_YEAR) {
    return false; // outside fixed table range
  }

  const uint32_t key = (uint32_t)year * 10000UL + (uint32_t)month * 100UL + (uint32_t)day;
  const uint16_t yearIndex = (uint16_t)(year - HOLIDAY_START_YEAR);
  const uint16_t base = yearIndex * HOLIDAYS_PER_YEAR;

  for (uint8_t i = 0; i < HOLIDAYS_PER_YEAR; ++i) {
    uint32_t d = pgm_read_dword(&NRW_HOLIDAYS_2026_2035[base + i]);
    if (d == key) {
      return true;
    }
  }
  return false;
}

uint8_t lastSundayOfMonth(uint16_t year, uint8_t month) {
  for (int d = 31; d >= 1; --d) {
    DateTime dt(year, month, (uint8_t)d, 0, 0, 0);
    if (dt.month() != month) {
      continue;
    }
    // RTClib: 0=Sunday, 1=Monday, ... 6=Saturday
    if (dt.dayOfTheWeek() == 0) {
      return (uint8_t)d;
    }
  }
  return 31;
}

bool isSummerTimeEU(const DateTime &localDateTime) {
  const uint16_t y = localDateTime.year();
  const uint8_t m = localDateTime.month();
  const uint8_t d = localDateTime.day();
  const uint8_t h = localDateTime.hour();

  if (m < 3 || m > 10) {
    return false;
  }
  if (m > 3 && m < 10) {
    return true;
  }

  if (m == 3) {
    const uint8_t lastSun = lastSundayOfMonth(y, 3);
    if (d > lastSun) return true;
    if (d < lastSun) return false;
    return (h >= 2); // from 02:00 local, summer time applies
  }

  // m == 10
  const uint8_t lastSun = lastSundayOfMonth(y, 10);
  if (d < lastSun) return true;
  if (d > lastSun) return false;
  return (h < 3); // until 03:00 local, still summer time
}

DayType determineDayType(const DateTime &dt) {
  const uint8_t dow = dt.dayOfTheWeek(); // 0=Sun, 6=Sat
  const bool weekend = (dow == 0 || dow == 6);
  const bool holiday = isHolidayNRW(dt.year(), dt.month(), dt.day());
  return (weekend || holiday) ? DAYTYPE_WEEKEND_OR_HOLIDAY : DAYTYPE_WEEKDAY;
}

void getAllowedIntervals(bool isSummer, DayType dayType, const Interval *&intervals, uint8_t &count) {
  if (isSummer) {
    if (dayType == DAYTYPE_WEEKDAY) {
      intervals = SUMMER_WEEKDAY_INTERVALS;
      count = sizeof(SUMMER_WEEKDAY_INTERVALS) / sizeof(SUMMER_WEEKDAY_INTERVALS[0]);
    } else {
      intervals = SUMMER_WEEKEND_HOL_INTERVALS;
      count = sizeof(SUMMER_WEEKEND_HOL_INTERVALS) / sizeof(SUMMER_WEEKEND_HOL_INTERVALS[0]);
    }
  } else {
    if (dayType == DAYTYPE_WEEKDAY) {
      intervals = WINTER_WEEKDAY_INTERVALS;
      count = sizeof(WINTER_WEEKDAY_INTERVALS) / sizeof(WINTER_WEEKDAY_INTERVALS[0]);
    } else {
      intervals = WINTER_WEEKEND_HOL_INTERVALS;
      count = sizeof(WINTER_WEEKEND_HOL_INTERVALS) / sizeof(WINTER_WEEKEND_HOL_INTERVALS[0]);
    }
  }
}

bool isAllowedNow(const DateTime &dt, bool isSummer, DayType dayType) {
  const Interval *intervals = nullptr;
  uint8_t count = 0;
  getAllowedIntervals(isSummer, dayType, intervals, count);

  const int32_t nowSec = secondsSinceMidnight(dt.hour(), dt.minute(), dt.second());
  for (uint8_t i = 0; i < count; ++i) {
    if (nowSec >= intervals[i].startSec && nowSec < intervals[i].endSec) {
      return true;
    }
  }
  return false;
}

int32_t secondsToEndOfCurrentAllowedInterval(const DateTime &dt, bool isSummer, DayType dayType) {
  const Interval *intervals = nullptr;
  uint8_t count = 0;
  getAllowedIntervals(isSummer, dayType, intervals, count);

  const int32_t nowSec = secondsSinceMidnight(dt.hour(), dt.minute(), dt.second());
  for (uint8_t i = 0; i < count; ++i) {
    if (nowSec >= intervals[i].startSec && nowSec < intervals[i].endSec) {
      return (int32_t)(intervals[i].endSec - nowSec);
    }
  }
  return -1;
}

void applyLedOutput(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#if USE_DEBUG_LED
  digitalWrite(DEBUG_LED_PIN, on ? HIGH : LOW);
#endif
}

void updateErrorBlink(unsigned long nowMs) {
  const bool on = ((nowMs / ERROR_BLINK_INTERVAL_MS) % 2UL) == 0UL;
  applyLedOutput(on);
}

void updateStateAndLed(const DateTime &nowLocal, unsigned long nowMs) {
  // Highest priority: RTC invalid (lost power / invalid time)
  if (rtcInvalidLatched) {
    currentState = LED_ERROR_RTC_INVALID;
    updateErrorBlink(nowMs);
    return;
  }

  const bool summer = isSummerTimeEU(nowLocal);
  const DayType dayType = determineDayType(nowLocal);
  const bool allowed = isAllowedNow(nowLocal, summer, dayType);
  const int32_t secToEnd = secondsToEndOfCurrentAllowedInterval(nowLocal, summer, dayType);

  // Priority after RTC error:
  // 1) forbidden 2) prewarn 3) allowed
  if (!allowed) {
    currentState = LED_ON_FORBIDDEN;
  } else if (secToEnd > 0 && secToEnd <= PREWARN_SECONDS) {
    currentState = LED_BLINK_PREWARN;
  } else {
    currentState = LED_OFF_ALLOWED;
  }

  switch (currentState) {
    case LED_OFF_ALLOWED:
      applyLedOutput(false);
      break;
    case LED_ON_FORBIDDEN:
      applyLedOutput(true);
      break;
    case LED_BLINK_PREWARN: {
        const bool on = ((nowMs / PREWARN_BLINK_INTERVAL_MS) % 2UL) == 0UL;
        applyLedOutput(on);
        break;
      }
    case LED_ERROR_RTC_INVALID:
      updateErrorBlink(nowMs);
      break;
    default:
      applyLedOutput(false);
      break;
  }

#if ENABLE_SERIAL_DEBUG
  if (nowLocal.second() != lastDebugSecond) {
    lastDebugSecond = nowLocal.second();
    const bool holiday = isHolidayNRW(nowLocal.year(), nowLocal.month(), nowLocal.day());
    Serial.print(nowLocal.year());
    Serial.print('-');
    if (nowLocal.month() < 10) Serial.print('0');
    Serial.print(nowLocal.month());
    Serial.print('-');
    if (nowLocal.day() < 10) Serial.print('0');
    Serial.print(nowLocal.day());
    Serial.print(' ');
    if (nowLocal.hour() < 10) Serial.print('0');
    Serial.print(nowLocal.hour());
    Serial.print(':');
    if (nowLocal.minute() < 10) Serial.print('0');
    Serial.print(nowLocal.minute());
    Serial.print(':');
    if (nowLocal.second() < 10) Serial.print('0');
    Serial.print(nowLocal.second());
    Serial.print(" holiday=");
    Serial.print(holiday ? "1" : "0");
    Serial.print(" summer=");
    Serial.print(summer ? "1" : "0");
    Serial.print(" allowed=");
    Serial.print(allowed ? "1" : "0");
    Serial.print(" state=");
    Serial.println((int)currentState);
  }
#endif
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
#if USE_DEBUG_LED
  pinMode(DEBUG_LED_PIN, OUTPUT);
#endif
  applyLedOutput(false);

#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  printSerialCommandHelp();
#endif

  Wire.begin();

  if (!rtc.begin()) {
    rtcAvailable = false;
    rtcInvalidLatched = true;
    currentState = LED_ERROR_RTC_INVALID;
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: DS3231 not found. RTC invalid."));
#endif
    return;
  }
  rtcAvailable = true;

#if SET_RTC_ON_UPLOAD
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#if ENABLE_SERIAL_DEBUG
  Serial.println(F("RTC adjusted to compile time."));
#endif
#endif

  if (rtc.lostPower()) {
    rtcInvalidLatched = true;
    currentState = LED_ERROR_RTC_INVALID;
#if ENABLE_SERIAL_DEBUG
    Serial.println(F("ERROR: RTC lost power, time invalid"));
#endif
  }
}

void loop() {
  const unsigned long nowMs = millis();
  processSerialCommands();

  if (!rtcAvailable) {
    currentState = LED_ERROR_RTC_INVALID;
    updateErrorBlink(nowMs);
#if ENABLE_SERIAL_DEBUG
    if ((nowMs - lastErrorSerialMs) >= ERROR_SERIAL_INTERVAL_MS) {
      lastErrorSerialMs = nowMs;
      Serial.println(F("ERROR: RTC lost power, time invalid"));
    }
#endif
    return;
  }

  if ((nowMs - lastRtcPollMs) < RTC_POLL_MS) {
    // keep blink smooth without blocking
    if (currentState == LED_BLINK_PREWARN) {
      const bool on = ((nowMs / PREWARN_BLINK_INTERVAL_MS) % 2UL) == 0UL;
      applyLedOutput(on);
    } else if (currentState == LED_ERROR_RTC_INVALID) {
      updateErrorBlink(nowMs);
    }
    return;
  }
  lastRtcPollMs = nowMs;

  if (rtc.lostPower()) {
    rtcInvalidLatched = true;
  }

  if (rtcInvalidLatched) {
    currentState = LED_ERROR_RTC_INVALID;
    updateErrorBlink(nowMs);
#if ENABLE_SERIAL_DEBUG
    if ((nowMs - lastErrorSerialMs) >= ERROR_SERIAL_INTERVAL_MS) {
      lastErrorSerialMs = nowMs;
      Serial.println(F("ERROR: RTC lost power, time invalid"));
    }
#endif
    return;
  }

  const DateTime nowLocal = rtc.now();
  updateStateAndLed(nowLocal, nowMs);
}
