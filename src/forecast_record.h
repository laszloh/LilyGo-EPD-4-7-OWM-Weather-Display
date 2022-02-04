#ifndef FORECAST_RECORD_H_
#define FORECAST_RECORD_H_

#include <Arduino.h>

enum class PressureTrend {
  same,
  rising,
  falling,
  zero
};

typedef struct { // For current Day and Day 1, 2, 3, etc
  int    Dt;
  String Icon;
  PressureTrend Trend;
  String Forecast0;
  String Description;
  float  Temperature;
  float  FeelsLike;
  float  DewPoint;
  float  Humidity;
  float  High;
  float  Low;
  float  Winddir;
  float  Windspeed;
  float  Rainfall;
  float  Snowfall;
  float  Pressure;
  int    Cloudcover;
  int    Visibility;
  int    Sunrise;
  int    Sunset;
  int    FTimezone;
  float  UVI;
} Forecast_record_type;

#endif /* ifndef FORECAST_RECORD_H_ */
