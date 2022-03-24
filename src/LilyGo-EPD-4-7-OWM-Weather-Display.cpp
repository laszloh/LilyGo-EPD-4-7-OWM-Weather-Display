#include "epd_driver.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zlib/zlib.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

#include <ArduinoJson.h>
#include <HTTPClient.h>

#include <SPI.h>
#include <WiFi.h>
#include <time.h>

#include "config.h"
#include "forecast_record.h"
#include "lang.h"

enum class Alignment { LEFT, RIGHT, CENTER };
enum class Color : uint8_t { White = 0xFF, LightGrey = 0xBB, Grey = 0x88, DarkGrey = 0x44, Black = 0x00 };

template <typename E> constexpr auto to_underlying(E e) noexcept {
    return static_cast<std::underlying_type_t<E>>(e);
}

constexpr bool LargeIcon = true;
constexpr bool SmallIcon = false;
constexpr int Large = 20;
constexpr int Small = 10;

int wifi_signal;
int vref = 1100;
struct tm timeinfo RTC_DATA_ATTR;

constexpr int max_readings = 24;

Forecast_record_type WxConditions;
Forecast_record_type WxForecast[max_readings];

float pressure_readings[max_readings] = {0};
float temperature_readings[max_readings] = {0};
float humidity_readings[max_readings] = {0};
float rain_readings[max_readings] = {0};
float snow_readings[max_readings] = {0};

constexpr uint32_t SleepDuration = 30 * 60; // in seconds
constexpr int WakeupHour = 7;
constexpr int SleepHour = 23;
uint32_t SleepTimer = 0;
constexpr long Delta = 30;


#include "moon.h"
#include "opensans10b.h"
#include "opensans12b.h"
#include "opensans18b.h"
#include "opensans24b.h"
#include "opensans8b.h"
#include "sunrise.h"
#include "sunset.h"
#include "uvi.h"

GFXfont currentFont;

constexpr size_t screenWidth = EPD_WIDTH;
constexpr size_t screenHeight = EPD_HEIGHT;

constexpr size_t frameBufferSize = (screenWidth * screenHeight) / 2;
uint8_t *framebuffer;

HTTPClient http;

void BeginSleep();
bool SetupTime();
uint8_t StartWiFi();
void StopWiFi();
void InitialiseSystem();
void loop();
void setup();
void Convert_Readings_to_Imperial();
bool DecodeWeather(WiFiClient &json, const bool forecast);
String ConvertUnixTime(int unix_time);
bool obtainWeatherData(WiFiClient &client, const bool forecast = true, const bool keepAlive = true);
constexpr float mm_to_inches(float value_mm);
constexpr float hPa_to_inHg(float value_hPa);
constexpr int JulianDate(int d, int m, int y);
constexpr float SumOfPrecip(float DataArray[], int readings);
String TitleCase(String text);
void DisplayWeather();
void DisplayGeneralInfoSection();
void DisplayWeatherIcon(int x, int y);
void DisplayMainWeatherSection(int x, int y);
void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius);
constexpr const char *WindDegToOrdinalDirection(float winddirection);
void DisplayTempHumiPressSection(int x, int y);
void DisplayForecastTextSection(int x, int y);
void DisplayVisiCCoverUVISection(int x, int y);
void Display_UVIndexLevel(int x, int y, float UVI);
void DisplayForecastWeather(int x, int y, int index, int fwidth);
constexpr double NormalizedMoonPhase(int d, int m, int y);
void DisplayAstronomySection(int x, int y);
void DrawMoon(int x, int y, int diameter, int dd, int mm, int yy, bool northenHemisphere);
constexpr const char *MoonPhase(int d, int m, int y, bool northenHemisphere);
void DisplayForecastSection(int x, int y);
void DisplayGraphSection(int x, int y);
void DisplayConditionsSection(int x, int y, String IconName, bool IconSize);
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength);
void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14);
void DrawPressureAndTrend(int x, int y, float pressure, PressureTrend slope);
void DisplayStatusSection(int x, int y, int rssi);
void DrawRSSI(int x, int y, int rssi);
String getDateString();
String getTimeString();
void DrawBattery(int x, int y);
void addcloud(int x, int y, int scale, int linesize);
void addrain(int x, int y, int scale, bool IconSize);
void addsnow(int x, int y, int scale, bool IconSize);
void addtstorm(int x, int y, int scale);
void addsun(int x, int y, int scale, bool IconSize);
void addfog(int x, int y, int scale, int linesize, bool IconSize);
void DrawAngledLine(int x, int y, int x1, int y1, int size, Color color);
void ClearSky(int x, int y, bool IconSize);
void BrokenClouds(int x, int y, bool IconSize);
void FewClouds(int x, int y, bool IconSize);
void ScatteredClouds(int x, int y, bool IconSize);
void Rain(int x, int y, bool IconSize);
void ChanceRain(int x, int y, bool IconSize);
void Thunderstorms(int x, int y, bool IconSize);
void Snow(int x, int y, bool IconSize);
void Mist(int x, int y, bool IconSize);
void CloudCover(int x, int y, int CloudCover);
void Visibility(int x, int y, String Visibility);
void addmoon(int x, int y, bool IconSize);
void Nodata(int x, int y, bool IconSize);
void DrawMoonImage(int x, int y);
void DrawSunriseImage(int x, int y);
void DrawSunsetImage(int x, int y);
void DrawUVI(int x, int y);
void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title,
               float DataArray[], int readings, bool auto_scale, bool barchart_mode);
void drawString(int x, int y, String text, Alignment align);
void fillCircle(int x, int y, int r, Color color);
void drawFastHLine(int16_t x0, int16_t y0, int length, Color color);
void drawFastVLine(int16_t x0, int16_t y0, int length, Color color);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color);
void drawCircle(int x0, int y0, int r, Color color);
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, Color color);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, Color color);
void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, Color color);
void drawPixel(int x, int y, Color color);
void setFont(GFXfont const &font);
void edp_update();

__attribute__((noreturn)) void BeginSleep() {
    epd_poweroff_all();
    uint32_t wakeTimeMs = millis();
    // modify SleepTimer for wake time
    SleepTimer = (SleepDuration * 1000) - wakeTimeMs;
    esp_sleep_enable_timer_wakeup(SleepTimer * 1000);
    log_d("Awake for : %d ms", wakeTimeMs);
    log_d("Entering %d (secs) of sleep time", SleepTimer / 1000);
    log_i("Starting deep-sleep period...");
    esp_deep_sleep_start();
}

bool SetupTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov");
    setenv("TZ", Timezone, 1);
    tzset();

    if(!getLocalTime(&timeinfo)) {
        log_e("Failed to obtain time");
        return false;
    }
    return true;
}

uint8_t StartWiFi() {
    log_i("Connecting to: %s", ssid);
    IPAddress dns(8, 8, 8, 8);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    if(WiFi.waitForConnectResult(5000) != WL_CONNECTED) {
        log_e("WiFi connection *** FAILED ***");
        WiFi.disconnect(true);
        return WiFi.status();
    }
    wifi_signal = WiFi.RSSI();
    log_i("WiFi connected at: %s", WiFi.localIP().toString().c_str());
    return WiFi.status();
}

void StopWiFi() {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    log_i("WiFi switched Off");
}

void InitialiseSystem() {
    Serial.begin(115200);
    log_i("Starting...");
    epd_init();

    log_d("Total heap: %d", ESP.getHeapSize());
    log_d("Free heap: %d", ESP.getFreeHeap());
    log_d("Total PSRAM: %d", ESP.getPsramSize());
    log_d("Free PSRAM: %d", ESP.getFreePsram());

    esp_reset_reason_t reason = esp_reset_reason();
    log_d("ESP reset reason: %d", reason);

    time_t now = time(NULL);
    log_v("local time: %d", time(NULL));

    char strftime_buf[64];
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    log_v("The current date/time is: %s", strftime_buf);

    framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), frameBufferSize);
    if(!framebuffer) {
        log_e("!!!!!!!!!!!!!!!!!!!!");
        log_e("Memory alloc failed!");
        log_e("!!!!!!!!!!!!!!!!!!!!");
        log_e("Going to sleep");
        BeginSleep();
    }
    memset(framebuffer, to_underlying(Color::White), frameBufferSize);
}

__attribute__((noreturn)) void loop() { BeginSleep(); }

__attribute__((noreturn)) void setup() {
    InitialiseSystem();
    if(StartWiFi() == WL_CONNECTED && SetupTime() == true) {
        bool WakeUp = false;
        if(WakeupHour > SleepHour)
            WakeUp = (timeinfo.tm_hour >= WakeupHour || timeinfo.tm_hour <= SleepHour);
        else
            WakeUp = (timeinfo.tm_hour >= WakeupHour && timeinfo.tm_hour <= SleepHour);
        if(WakeUp) {
            byte Attempts = 1;
            bool RxWeather = false;
            bool RxForecast = false;
            WiFiClientSecure client;
            client.setCACert(caCertOWM);
            while((RxWeather == false || RxForecast == false) && Attempts <= 2) {
                if(RxWeather == false)
                    RxWeather = obtainWeatherData(client, false);
                if(RxForecast == false)
                    RxForecast = obtainWeatherData(client);
                Attempts++;
            }
            client.stop();
            log_i("Received all weather data...");
            if(RxWeather && RxForecast) {
                StopWiFi();
                epd_poweron();
                epd_clear();
                DisplayWeather();
                edp_update();
                epd_poweroff_all();
            }
        }
    }
    BeginSleep();
}

void Convert_Readings_to_Imperial() {
    WxConditions.Pressure = hPa_to_inHg(WxConditions.Pressure);
    WxForecast[0].Rainfall = mm_to_inches(WxForecast[0].Rainfall);
    WxForecast[0].Snowfall = mm_to_inches(WxForecast[0].Snowfall);
}

bool DecodeWeather(WiFiClient &json, const bool forecast) {
    log_i("Deserializing weather data");
    DynamicJsonDocument doc(64 * 1024);
    DeserializationError error = deserializeJson(doc, json);
    if(error) {
        log_e("DeserializeJson() failed: %s (%d)", error.c_str(), error);
        return false;
    }

    JsonObject root = doc.as<JsonObject>();
    log_d("Decoding %s data", (forecast ? "forecast" : "oncall"));
    if(!forecast) {

        WxConditions.High = -50;
        WxConditions.Low = 50;
        WxConditions.FTimezone = doc["timezone_offset"];
        JsonObject current = doc["current"];
        WxConditions.Sunrise = current["sunrise"];
        log_v("   SRis: %d", WxConditions.Sunrise);
        WxConditions.Sunset = current["sunset"];
        log_v("   SSet: %d", WxConditions.Sunset);
        WxConditions.Temperature = current["temp"];
        log_v("   Temp: %.2f", WxConditions.Temperature);
        WxConditions.FeelsLike = current["feels_like"];
        log_v("   FLik: %.2f", WxConditions.FeelsLike);
        WxConditions.Pressure = current["pressure"];
        log_v("   Pres: %.2f", WxConditions.Pressure);
        WxConditions.Humidity = current["humidity"];
        log_v("   Humi: %.2f", WxConditions.Humidity);
        WxConditions.DewPoint = current["dew_point"];
        log_v("   DPoi: %.2f", WxConditions.DewPoint);
        WxConditions.UVI = current["uvi"];
        log_v("   UVin: %.2f", WxConditions.UVI);
        WxConditions.Cloudcover = current["clouds"];
        log_v("   CCov: %d", WxConditions.Cloudcover);
        WxConditions.Visibility = current["visibility"];
        log_v("   Visi: %d", WxConditions.Visibility);
        WxConditions.Windspeed = current["wind_speed"];
        log_v("   WSpd: %.2f", WxConditions.Windspeed);
        WxConditions.Winddir = current["wind_deg"];
        log_v("   WDir: %.2f", WxConditions.Winddir);
        JsonObject current_weather = current["weather"][0];
        String Description = current_weather["description"];
        String Icon = current_weather["icon"];
        WxConditions.Forecast0 = Description;
        log_v("   Fore: %s", WxConditions.Forecast0.c_str());
        WxConditions.Icon = Icon;
        log_v("   Icon: %s", WxConditions.Icon.c_str());
    } else {
        log_d("Receiving Forecast period - ");
        JsonArray list = root["list"];
        for(auto r = 0; r < max_readings; r++) {
            log_v("   Period-%d--------------", r);
            WxForecast[r].Dt = list[r]["dt"].as<int>();
            WxForecast[r].Temperature = list[r]["main"]["temp"].as<float>();
            log_v("   Temp: %.2f", WxForecast[r].Temperature);
            WxForecast[r].Low = list[r]["main"]["temp_min"].as<float>();
            log_v("   TLow: %.2f", WxForecast[r].Low);
            WxForecast[r].High = list[r]["main"]["temp_max"].as<float>();
            log_v("   THig: %.2f", WxForecast[r].High);
            WxForecast[r].Pressure = list[r]["main"]["pressure"].as<float>();
            log_v("   Pres: %.2f", WxForecast[r].Pressure);
            WxForecast[r].Humidity = list[r]["main"]["humidity"].as<float>();
            log_v("   Humi: %.2f", WxForecast[r].Humidity);
            WxForecast[r].Icon = list[r]["weather"][0]["icon"].as<const char *>();
            log_v("   Icon: %s", WxForecast[r].Icon.c_str());
            WxForecast[r].Rainfall = list[r]["rain"]["3h"].as<float>();
            log_v("   Rain: %.2f", WxForecast[r].Rainfall);
            WxForecast[r].Snowfall = list[r]["snow"]["3h"].as<float>();
            log_v("   Snow: %.2f", WxForecast[r].Snowfall);
            if(r < 8) {
                if(WxForecast[r].High > WxConditions.High)
                    WxConditions.High = WxForecast[r].High;
                if(WxForecast[r].Low < WxConditions.Low)
                    WxConditions.Low = WxForecast[r].Low;
            }
        }

        float pressure_trend = WxForecast[0].Pressure - WxForecast[2].Pressure;
        pressure_trend = ((int)(pressure_trend * 10)) / 10.0;
        WxConditions.Trend = PressureTrend::same;
        if(pressure_trend > 0)
            WxConditions.Trend = PressureTrend::rising;
        if(pressure_trend < 0)
            WxConditions.Trend = PressureTrend::falling;
        if(pressure_trend == 0)
            WxConditions.Trend = PressureTrend::zero;

        if(!Metric)
            Convert_Readings_to_Imperial();
    }
    return true;
}

String ConvertUnixTime(int unix_time) {

    time_t tm = unix_time;
    struct tm *now_tm = localtime(&tm);
    char output[40];
    if(Metric) {
        strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
    } else {
        strftime(output, sizeof(output), "%I:%M%P %m/%d/%y", now_tm);
    }
    return output;
}

bool obtainWeatherData(WiFiClient &client, const bool forecast, const bool keepAlive) {
    constexpr const char *units = (Metric ? "metric" : "imperial");
    const String forecastRequest = String("/data/2.5/forecast?lat=") + Latitude + "&lon=" + Longitude
        + "&appid=" + apikey + "&mode=json&units=" + units + "&lang=" + Language;
    const String oncallRequest = String("/data/2.5/onecall?lat=") + Latitude + "&lon=" + Longitude
        + "&appid=" + apikey + "&mode=json&units=" + units + "&lang=" + Language
        + "&exclude=minutely,hourly,alerts,daily";

    bool ret = true;
    http.setReuse(keepAlive);

    const char *uri = (forecast) ? forecastRequest.c_str() : oncallRequest.c_str();
    log_v("HTTPS request: %s", uri);
    http.begin(client, server, 443, uri, true);
    int httpCode = http.GET();
    if(httpCode == HTTP_CODE_OK) {
        ret = DecodeWeather(http.getStream(), forecast);
    } else {
        log_e("connection failed, error: %s (%d)", http.errorToString(httpCode).c_str(), httpCode);
        ret = false;
    }
    http.end();

    return ret;
}

constexpr float mm_to_inches(float value_mm) { return 0.0393701 * value_mm; }

constexpr float hPa_to_inHg(float value_hPa) { return 0.02953 * value_hPa; }

constexpr int JulianDate(int d, int m, int y) {
    int yy = y - (int)((12 - m) / 10);
    int mm = m + 9;
    if(mm >= 12)
        mm = mm - 12;
    int k1 = (int)(365.25 * (yy + 4712));
    int k2 = (int)(30.6001 * mm + 0.5);
    int k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;

    int j = k1 + k2 + d + 59 + 1;
    if(j > 2299160)
        j = j - k3;
    return j;
}

constexpr float SumOfPrecip(float DataArray[], int readings) {
    float sum = 0;
    for(int i = 0; i <= readings; i++)
        sum += DataArray[i];
    return sum;
}

String TitleCase(String text) {
    if(text.length() > 0) {
        String temp_text = text.substring(0, 1);
        temp_text.toUpperCase();
        return temp_text + text.substring(1);
    } else
        return text;
}

void DisplayWeather() {
    DisplayStatusSection(600, 20, wifi_signal);
    DisplayGeneralInfoSection();
    DisplayDisplayWindSection(137, 150, WxConditions.Winddir, WxConditions.Windspeed, 100);
    DisplayAstronomySection(5, 252);
    DisplayMainWeatherSection(320, 110);
    DisplayWeatherIcon(835, 140);
    DisplayForecastSection(285, 220);
    DisplayGraphSection(320, 220);
}

void DisplayGeneralInfoSection() {
    setFont(OpenSans10B);
    drawString(5, 2, City, Alignment::LEFT);
    setFont(OpenSans8B);
    drawString(500, 2, getDateString() + " @ " + getTimeString(), Alignment::LEFT);
}

void DisplayWeatherIcon(int x, int y) { DisplayConditionsSection(x, y, WxConditions.Icon, LargeIcon); }

void DisplayMainWeatherSection(int x, int y) {
    setFont(OpenSans8B);
    DisplayTempHumiPressSection(x, y - 60);
    DisplayForecastTextSection(x - 55, y + 45);
    DisplayVisiCCoverUVISection(x - 10, y + 95);
}

void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius) {
    arrow(x, y, Cradius - 22, angle, 18, 33);
    setFont(OpenSans8B);
    int dxo, dyo, dxi, dyi;
    drawCircle(x, y, Cradius, Color::Grey);
    drawCircle(x, y, Cradius + 1, Color::Grey);
    drawCircle(x, y, Cradius * 0.7, Color::Grey);
    for(float a = 0; a < 360; a = a + 22.5) {
        dxo = Cradius * cos((a - 90) * PI / 180);
        dyo = Cradius * sin((a - 90) * PI / 180);
        if(a == 45)
            drawString(dxo + x + 15, dyo + y - 18, TXT_NE, Alignment::CENTER);
        if(a == 135)
            drawString(dxo + x + 20, dyo + y - 2, TXT_SE, Alignment::CENTER);
        if(a == 225)
            drawString(dxo + x - 20, dyo + y - 2, TXT_SW, Alignment::CENTER);
        if(a == 315)
            drawString(dxo + x - 15, dyo + y - 18, TXT_NW, Alignment::CENTER);
        dxi = dxo * 0.9;
        dyi = dyo * 0.9;
        drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Color::Grey);
        dxo = dxo * 0.7;
        dyo = dyo * 0.7;
        dxi = dxo * 0.9;
        dyi = dyo * 0.9;
        drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Color::Grey);
    }
    drawString(x, y - Cradius - 20, TXT_N, Alignment::CENTER);
    drawString(x, y + Cradius + 10, TXT_S, Alignment::CENTER);
    drawString(x - Cradius - 15, y - 5, TXT_W, Alignment::CENTER);
    drawString(x + Cradius + 10, y - 5, TXT_E, Alignment::CENTER);
    drawString(x + 3, y + 50, String(angle, 0) + "°", Alignment::CENTER);
    setFont(OpenSans12B);
    drawString(x, y - 50, WindDegToOrdinalDirection(angle), Alignment::CENTER);
    setFont(OpenSans24B);
    drawString(x + 3, y - 18, String(windspeed, 1), Alignment::CENTER);
    setFont(OpenSans12B);
    drawString(x, y + 25, (Metric ? "m/s" : "mph"), Alignment::CENTER);
}

constexpr const char *WindDegToOrdinalDirection(float winddirection) {
    if(winddirection >= 348.75 || winddirection < 11.25)
        return TXT_N;
    if(winddirection >= 11.25 && winddirection < 33.75)
        return TXT_NNE;
    if(winddirection >= 33.75 && winddirection < 56.25)
        return TXT_NE;
    if(winddirection >= 56.25 && winddirection < 78.75)
        return TXT_ENE;
    if(winddirection >= 78.75 && winddirection < 101.25)
        return TXT_E;
    if(winddirection >= 101.25 && winddirection < 123.75)
        return TXT_ESE;
    if(winddirection >= 123.75 && winddirection < 146.25)
        return TXT_SE;
    if(winddirection >= 146.25 && winddirection < 168.75)
        return TXT_SSE;
    if(winddirection >= 168.75 && winddirection < 191.25)
        return TXT_S;
    if(winddirection >= 191.25 && winddirection < 213.75)
        return TXT_SSW;
    if(winddirection >= 213.75 && winddirection < 236.25)
        return TXT_SW;
    if(winddirection >= 236.25 && winddirection < 258.75)
        return TXT_WSW;
    if(winddirection >= 258.75 && winddirection < 281.25)
        return TXT_W;
    if(winddirection >= 281.25 && winddirection < 303.75)
        return TXT_WNW;
    if(winddirection >= 303.75 && winddirection < 326.25)
        return TXT_NW;
    if(winddirection >= 326.25 && winddirection < 348.75)
        return TXT_NNW;
    return "?";
}

void DisplayTempHumiPressSection(int x, int y) {
    setFont(OpenSans18B);
    drawString(x - 30, y,
               String(WxConditions.Temperature, 1) + "°   " + String(WxConditions.Humidity, 0) + "%",
               Alignment::LEFT);
    setFont(OpenSans12B);
    DrawPressureAndTrend(x + 195, y + 15, WxConditions.Pressure, WxConditions.Trend);
    int Yoffset = 42;
    if(WxConditions.Windspeed > 0) {
        drawString(x - 30, y + Yoffset, String(WxConditions.FeelsLike, 1) + "° FL", Alignment::LEFT);
        Yoffset += 30;
    }
    drawString(x - 30, y + Yoffset,
               String(WxConditions.High, 0) + "° | " + String(WxConditions.Low, 0) + "° Hi/Lo",
               Alignment::LEFT);
}

void DisplayForecastTextSection(int x, int y) {
#define lineWidth 34
    setFont(OpenSans12B);
    String Wx_Description = WxConditions.Forecast0;
    Wx_Description.replace(".", "");
    int spaceRemaining = 0, p = 0, charCount = 0, Width = lineWidth;
    while(p < Wx_Description.length()) {
        if(Wx_Description.substring(p, p + 1) == " ")
            spaceRemaining = p;
        if(charCount > Width - 1) {
            Wx_Description = Wx_Description.substring(0, spaceRemaining) + "~"
                + Wx_Description.substring(spaceRemaining + 1);
            charCount = 0;
        }
        p++;
        charCount++;
    }
    if(WxForecast[0].Rainfall > 0)
        Wx_Description += " (" + String(WxForecast[0].Rainfall, 1) + String((Metric ? "mm" : "in")) + ")";
    String Line1 = Wx_Description.substring(0, Wx_Description.indexOf("~"));
    String Line2 = Wx_Description.substring(Wx_Description.indexOf("~") + 1);
    drawString(x + 30, y + 5, TitleCase(Line1), Alignment::LEFT);
    if(Line1 != Line2)
        drawString(x + 30, y + 30, Line2, Alignment::LEFT);
}

void DisplayVisiCCoverUVISection(int x, int y) {
    setFont(OpenSans12B);
    log_v("==========================");
    log_v("Visibility: %d", WxConditions.Visibility);
    Visibility(x + 5, y, String(WxConditions.Visibility) + "M");
    CloudCover(x + 155, y, WxConditions.Cloudcover);
    Display_UVIndexLevel(x + 265, y, WxConditions.UVI);
}

void Display_UVIndexLevel(int x, int y, float UVI) {
    String Level = "";
    if(UVI <= 2)
        Level = " (L)";
    if(UVI >= 3 && UVI <= 5)
        Level = " (M)";
    if(UVI >= 6 && UVI <= 7)
        Level = " (H)";
    if(UVI >= 8 && UVI <= 10)
        Level = " (VH)";
    if(UVI >= 11)
        Level = " (EX)";
    drawString(x + 20, y - 5, String(UVI, (UVI < 0 ? 1 : 0)) + Level, Alignment::LEFT);
    DrawUVI(x - 10, y - 5);
}

void DisplayForecastWeather(int x, int y, int index, int fwidth) {
    x = x + fwidth * index;
    DisplayConditionsSection(x + fwidth / 2 - 5, y + 85, WxForecast[index].Icon, SmallIcon);
    setFont(OpenSans10B);
    drawString(x + fwidth / 2, y + 30,
               String(ConvertUnixTime(WxForecast[index].Dt + WxConditions.FTimezone).substring(0, 5)),
               Alignment::CENTER);
    drawString(x + fwidth / 2, y + 130,
               String(WxForecast[index].High, 0) + "°/" + String(WxForecast[index].Low, 0) + "°",
               Alignment::CENTER);
}

constexpr double NormalizedMoonPhase(int d, int m, int y) {
    int j = JulianDate(d, m, y);

    double Phase = (j + 4.867) / 29.53059;
    return (Phase - (int)Phase);
}

void DisplayAstronomySection(int x, int y) {
    setFont(OpenSans10B);
    time_t now = time(NULL);
    struct tm *now_utc = gmtime(&now);
    drawString(x + 5, y + 102,
               MoonPhase(now_utc->tm_mday, now_utc->tm_mon + 1, now_utc->tm_year + 1900, NorthenHemisphere),
               Alignment::LEFT);
    DrawMoonImage(x + 10, y + 23);
    DrawMoon(x - 28, y - 15, 75, now_utc->tm_mday, now_utc->tm_mon + 1, now_utc->tm_year + 1900,
             NorthenHemisphere);
    drawString(x + 115, y + 40, ConvertUnixTime(WxConditions.Sunrise).substring(0, 5), Alignment::LEFT);
    drawString(x + 115, y + 80, ConvertUnixTime(WxConditions.Sunset).substring(0, 5), Alignment::LEFT);
    DrawSunriseImage(x + 180, y + 20);
    DrawSunsetImage(x + 180, y + 60);
}

void DrawMoon(int x, int y, int diameter, int dd, int mm, int yy, bool northenHemisphere) {
    double Phase = NormalizedMoonPhase(dd, mm, yy);
    if(!northenHemisphere) // we are in the southern
        Phase = 1 - Phase;

    fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, Color::DarkGrey);
    const int number_of_lines = 90;
    for(double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++) {
        double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);

        double Rpos = 2 * Xpos;
        double Xpos1, Xpos2;
        if(Phase < 0.5) {
            Xpos1 = -Xpos;
            Xpos2 = Rpos - 2 * Phase * Rpos - Xpos;
        } else {
            Xpos1 = Xpos;
            Xpos2 = Xpos - 2 * Phase * Rpos + Rpos;
        }

        double pW1x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
        double pW1y = (number_of_lines - Ypos) / number_of_lines * diameter + y;
        double pW2x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
        double pW2y = (number_of_lines - Ypos) / number_of_lines * diameter + y;
        double pW3x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
        double pW3y = (Ypos + number_of_lines) / number_of_lines * diameter + y;
        double pW4x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
        double pW4y = (Ypos + number_of_lines) / number_of_lines * diameter + y;
        drawLine(pW1x, pW1y, pW2x, pW2y, Color::White);
        drawLine(pW3x, pW3y, pW4x, pW4y, Color::White);
    }
    drawCircle(x + diameter - 1, y + diameter, diameter / 2, Color::Grey);
}

constexpr const char *MoonPhase(int d, int m, int y, bool northenHemisphere) {
    if(m < 3) {
        y--;
        m += 12;
    }
    ++m;
    int c = 365.25 * y;
    int e = 30.6 * m;
    double jd = c + e + d - 694039.09;
    jd /= 29.53059;
    int b = jd;
    jd -= b;
    b = jd * 8 + 0.5;
    b = b & 7;
    if(!northenHemisphere)
        b = 7 - b;
    if(b == 0)
        return TXT_MOON_NEW;
    if(b == 1)
        return TXT_MOON_WAXING_CRESCENT;
    if(b == 2)
        return TXT_MOON_FIRST_QUARTER;
    if(b == 3)
        return TXT_MOON_WAXING_GIBBOUS;
    if(b == 4)
        return TXT_MOON_FULL;
    if(b == 5)
        return TXT_MOON_WANING_GIBBOUS;
    if(b == 6)
        return TXT_MOON_THIRD_QUARTER;
    if(b == 7)
        return TXT_MOON_WANING_CRESCENT;
    return "";
}

void DisplayForecastSection(int x, int y) {
    int f = 0;
    do {
        DisplayForecastWeather(x, y, f, 82);
        f++;
    } while(f < 8);
}

void DisplayGraphSection(int x, int y) {
    int r = 0;
    do {
        pressure_readings[r] = (Metric ? WxForecast[r].Pressure : hPa_to_inHg(WxForecast[r].Pressure));
        rain_readings[r] = (Metric ? WxForecast[r].Rainfall : mm_to_inches(WxForecast[r].Rainfall));
        snow_readings[r] = (Metric ? WxForecast[r].Snowfall : mm_to_inches(WxForecast[r].Snowfall));
        temperature_readings[r] = WxForecast[r].Temperature;
        humidity_readings[r] = WxForecast[r].Humidity;
        r++;
    } while(r < max_readings);
    int gwidth = 175, gheight = 100;
    int gx = (screenWidth - gwidth * 4) / 5 + 8;
    int gy = (screenHeight - gheight - 30);
    int gap = gwidth + gx;

    DrawGraph(gx + 0 * gap, gy, gwidth, gheight, 900, 1050, Metric ? TXT_PRESSURE_HPA : TXT_PRESSURE_IN,
              pressure_readings, max_readings, true, false);
    DrawGraph(gx + 1 * gap, gy, gwidth, gheight, 10, 30, Metric ? TXT_TEMPERATURE_C : TXT_TEMPERATURE_F,
              temperature_readings, max_readings, true, false);
    DrawGraph(gx + 2 * gap, gy, gwidth, gheight, 0, 100, TXT_HUMIDITY_PERCENT, humidity_readings,
              max_readings, false, false);
    if(SumOfPrecip(rain_readings, max_readings) >= SumOfPrecip(snow_readings, max_readings))
        DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Metric ? TXT_RAINFALL_MM : TXT_RAINFALL_IN,
                  rain_readings, max_readings, true, true);
    else
        DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Metric ? TXT_SNOWFALL_MM : TXT_SNOWFALL_IN,
                  snow_readings, max_readings, true, true);
}

void DisplayConditionsSection(int x, int y, String IconName, bool IconSize) {
    log_v("Icon name: %s", IconName);

    if(IconName.endsWith("n"))
        addmoon(x, y, IconSize);

    IconName = IconName.substring(0, 2);

    if(IconName == "01")
        ClearSky(x, y, IconSize);
    else if(IconName == "02")
        FewClouds(x, y, IconSize);
    else if(IconName == "03")
        ScatteredClouds(x, y, IconSize);
    else if(IconName == "04")
        BrokenClouds(x, y, IconSize);
    else if(IconName == "09")
        ChanceRain(x, y, IconSize);
    else if(IconName == "10")
        Rain(x, y, IconSize);
    else if(IconName == "11")
        Thunderstorms(x, y, IconSize);
    else if(IconName == "13")
        Snow(x, y, IconSize);
    else if(IconName == "50")
        Mist(x, y, IconSize);
    else
        Nodata(x, y, IconSize);
}

void arrow(int x, int y, int asize, float aangle, int pwidth, int plength) {
    float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x;
    float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y;
    float x1 = 0;
    float y1 = plength;
    float x2 = pwidth / 2;
    float y2 = pwidth / 2;
    float x3 = -pwidth / 2;
    float y3 = pwidth / 2;
    float angle = aangle * PI / 180 - 135;
    float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
    float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
    float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
    float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
    float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
    float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
    fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, Color::Grey);
}

void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14) {
    drawLine(x + o1, y + o2, x + o3, y + o4, Color::Grey);
    drawLine(x + o11, y + o12, x + o13, y + o14, Color::Grey);
}

void DrawPressureAndTrend(int x, int y, float pressure, PressureTrend slope) {
    drawString(x + 25, y - 10, String(pressure, (Metric ? 0 : 1)) + (Metric ? "hPa" : "in"), Alignment::LEFT);
    if(slope == PressureTrend::rising) {
        DrawSegment(x, y, 0, 0, 8, -8, 8, -8, 16, 0);
        DrawSegment(x - 1, y, 0, 0, 8, -8, 8, -8, 16, 0);
    } else if(slope == PressureTrend::zero) {
        DrawSegment(x, y, 8, -8, 16, 0, 8, 8, 16, 0);
        DrawSegment(x - 1, y, 8, -8, 16, 0, 8, 8, 16, 0);
    } else if(slope == PressureTrend::falling) {
        DrawSegment(x, y, 0, 0, 8, 8, 8, 8, 16, 0);
        DrawSegment(x - 1, y, 0, 0, 8, 8, 8, 8, 16, 0);
    }
}

void DisplayStatusSection(int x, int y, int rssi) {
    setFont(OpenSans8B);
    DrawRSSI(x + 305, y + 15, rssi);
    DrawBattery(x + 150, y);
}

void DrawRSSI(int x, int y, int rssi) {
    int WIFIsignal = 0;
    int xpos = 1;
    for(int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20) {
        if(_rssi <= -20)
            WIFIsignal = 30;
        if(_rssi <= -40)
            WIFIsignal = 24;
        if(_rssi <= -60)
            WIFIsignal = 18;
        if(_rssi <= -80)
            WIFIsignal = 12;
        if(_rssi <= -100)
            WIFIsignal = 6;
        fillRect(x + xpos * 8, y - WIFIsignal, 6, WIFIsignal, Color::Grey);
        xpos++;
    }
}

String getDateString() {
    constexpr const char *metricDate = "%s, %02u %s %04u";
    constexpr const char *imperialDate = "%a %b-%d-%Y";

    char buf[64];
    strftime(buf, sizeof(buf), (Metric) ? metricDate : imperialDate, &timeinfo);
    return buf;
}

String getTimeString() {
    constexpr const char *metricTime = "%H:%M:%S";
    constexpr const char *imperialTime = "%r";

    char buf[64];
    strftime(buf, sizeof(buf), (Metric) ? metricTime : imperialTime, &timeinfo);
    return buf;
}

void DrawBattery(int x, int y) {
    uint8_t percentage = 100;
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type
        = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    if(val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        log_d("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    }
    float voltage = analogRead(36) / 4096.0 * 6.566 * (vref / 1000.0);
    if(voltage > 1) {
        log_d("Voltage: %.2f", voltage);
        percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3)
            + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
        if(voltage >= 4.20)
            percentage = 100;
        if(voltage <= 3.20)
            percentage = 0;
        drawRect(x + 25, y - 14, 40, 15, Color::Grey);
        fillRect(x + 65, y - 10, 4, 7, Color::Grey);
        fillRect(x + 27, y - 12, 36 * percentage / 100.0, 11, Color::Grey);
        drawString(x + 85, y - 14, String(percentage) + "%  " + String(voltage, 1) + "v", Alignment::LEFT);
    }
}


void addcloud(int x, int y, int scale, int linesize) {
    fillCircle(x - scale * 3, y, scale, Color::Grey);
    fillCircle(x + scale * 3, y, scale, Color::Grey);
    fillCircle(x - scale, y - scale, scale * 1.4, Color::Grey);
    fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, Color::Grey);
    fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, Color::Grey);
    fillCircle(x - scale * 3, y, scale - linesize, Color::White);
    fillCircle(x + scale * 3, y, scale - linesize, Color::White);
    fillCircle(x - scale, y - scale, scale * 1.4 - linesize, Color::White);
    fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, Color::White);
    fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2,
             Color::White);
}

void addrain(int x, int y, int scale, bool IconSize) {
    if(IconSize == SmallIcon) {
        setFont(OpenSans8B);
        drawString(x - 25, y + 12, "///////", Alignment::LEFT);
    } else {
        setFont(OpenSans18B);
        drawString(x - 60, y + 25, "///////", Alignment::LEFT);
    }
}

void addsnow(int x, int y, int scale, bool IconSize) {
    if(IconSize == SmallIcon) {
        setFont(OpenSans8B);
        drawString(x - 25, y + 15, "* * * *", Alignment::LEFT);
    } else {
        setFont(OpenSans18B);
        drawString(x - 60, y + 30, "* * * *", Alignment::LEFT);
    }
}

void addtstorm(int x, int y, int scale) {
    y = y + scale / 2;
    for(int i = 1; i < 5; i++) {
        drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0,
                 y + scale, Color::Grey);
        drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1,
                 y + scale, Color::Grey);
        drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2,
                 y + scale, Color::Grey);
        drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0,
                 y + scale * 1.5 + 0, Color::Grey);
        drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0,
                 y + scale * 1.5 + 1, Color::Grey);
        drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0,
                 y + scale * 1.5 + 2, Color::Grey);
        drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0,
                 y + scale * 1.5, Color::Grey);
        drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1,
                 y + scale * 1.5, Color::Grey);
        drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2,
                 y + scale * 1.5, Color::Grey);
    }
}

void addsun(int x, int y, int scale, bool IconSize) {
    int linesize = 5;
    fillRect(x - scale * 2, y, scale * 4, linesize, Color::Grey);
    fillRect(x, y - scale * 2, linesize, scale * 4, Color::Grey);
    DrawAngledLine(x + scale * 1.4, y + scale * 1.4, (x - scale * 1.4), (y - scale * 1.4), linesize * 1.5,
                   Color::Grey);
    DrawAngledLine(x - scale * 1.4, y + scale * 1.4, (x + scale * 1.4), (y - scale * 1.4), linesize * 1.5,
                   Color::Grey);
    fillCircle(x, y, scale * 1.3, Color::White);
    fillCircle(x, y, scale, Color::Grey);
    fillCircle(x, y, scale - linesize, Color::White);
}

void addfog(int x, int y, int scale, int linesize, bool IconSize) {
    if(IconSize == SmallIcon)
        linesize = 3;
    for(int i = 0; i < 6; i++) {
        fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, Color::Grey);
        fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, Color::Grey);
        fillRect(x - scale * 3, y + scale * 2.5, scale * 6, linesize, Color::Grey);
    }
}

void DrawAngledLine(int x, int y, int x1, int y1, int size, Color color) {
    int dx = (size / 2.0) * (x - x1) / sqrt(sq(x - x1) + sq(y - y1));
    int dy = (size / 2.0) * (y - y1) / sqrt(sq(x - x1) + sq(y - y1));
    fillTriangle(x + dx, y - dy, x - dx, y + dy, x1 + dx, y1 - dy, color);
    fillTriangle(x - dx, y + dy, x1 - dx, y1 + dy, x1 + dx, y1 - dy, color);
}

void ClearSky(int x, int y, bool IconSize) {
    int scale = Small;
    if(IconSize == LargeIcon)
        scale = Large;
    y += (IconSize ? 0 : 10);
    addsun(x, y, scale * (IconSize ? 1.7 : 1.2), IconSize);
}

void BrokenClouds(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    y += 15;
    if(IconSize == LargeIcon)
        scale = Large;
    addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
    addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
}

void FewClouds(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    y += 15;
    if(IconSize == LargeIcon)
        scale = Large;
    addcloud(x + (IconSize ? 10 : 0), y, scale * (IconSize ? 0.9 : 0.8), linesize);
    addsun((x + (IconSize ? 10 : 0)) - scale * 1.8, y - scale * 1.6, scale, IconSize);
}

void ScatteredClouds(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    y += 15;
    if(IconSize == LargeIcon)
        scale = Large;
    addcloud(x - (IconSize ? 35 : 0), y * (IconSize ? 0.75 : 0.93), scale / 2, linesize);
    addcloud(x, y, scale * 0.9, linesize);
}

void Rain(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    y += 15;
    if(IconSize == LargeIcon)
        scale = Large;
    addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
    addrain(x, y, scale, IconSize);
}

void ChanceRain(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    if(IconSize == LargeIcon)
        scale = Large;
    y += 15;
    addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
    addcloud(x, y, scale * (IconSize ? 1 : 0.65), linesize);
    addrain(x, y, scale, IconSize);
}

void Thunderstorms(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    if(IconSize == LargeIcon)
        scale = Large;
    y += 5;
    addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
    addtstorm(x, y, scale);
}

void Snow(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    if(IconSize == LargeIcon)
        scale = Large;
    addcloud(x, y, scale * (IconSize ? 1 : 0.75), linesize);
    addsnow(x, y, scale, IconSize);
}

void Mist(int x, int y, bool IconSize) {
    int scale = Small, linesize = 5;
    if(IconSize == LargeIcon)
        scale = Large;
    addsun(x, y, scale * (IconSize ? 1 : 0.75), linesize);
    addfog(x, y, scale, linesize, IconSize);
}

void CloudCover(int x, int y, int CloudCover) {
    addcloud(x - 9, y, Small * 0.3, 2);
    addcloud(x + 3, y - 2, Small * 0.3, 2);
    addcloud(x, y + 15, Small * 0.6, 2);
    drawString(x + 30, y, String(CloudCover) + "%", Alignment::LEFT);
}

void Visibility(int x, int y, String Visibility) {
    float start_angle = 0.52, end_angle = 2.61, Offset = 10;
    int r = 14;
    for(float i = start_angle; i < end_angle; i = i + 0.05) {
        drawPixel(x + r * cos(i), y - r / 2 + r * sin(i) + Offset, Color::Grey);
        drawPixel(x + r * cos(i), 1 + y - r / 2 + r * sin(i) + Offset, Color::Grey);
    }
    start_angle = 3.61;
    end_angle = 5.78;
    for(float i = start_angle; i < end_angle; i = i + 0.05) {
        drawPixel(x + r * cos(i), y + r / 2 + r * sin(i) + Offset, Color::Grey);
        drawPixel(x + r * cos(i), 1 + y + r / 2 + r * sin(i) + Offset, Color::Grey);
    }
    fillCircle(x, y + Offset, r / 4, Color::Grey);
    drawString(x + 20, y, Visibility, Alignment::LEFT);
}

void addmoon(int x, int y, bool IconSize) {
    int xOffset = 65;
    int yOffset = 12;
    if(IconSize == LargeIcon) {
        xOffset = 130;
        yOffset = -40;
    }
    fillCircle(x - 28 + xOffset, y - 37 + yOffset, uint16_t(Small * 1.0), Color::Grey);
    fillCircle(x - 16 + xOffset, y - 37 + yOffset, uint16_t(Small * 1.6), Color::White);
}

void Nodata(int x, int y, bool IconSize) {
    if(IconSize == LargeIcon)
        setFont(OpenSans24B);
    else
        setFont(OpenSans12B);
    drawString(x - 3, y - 10, "?", Alignment::CENTER);
}

void DrawMoonImage(int x, int y) {
    Rect_t area = {.x = x, .y = y, .width = moon_width, .height = moon_height};
    epd_draw_grayscale_image(area, (uint8_t *)moon_data);
}

void DrawSunriseImage(int x, int y) {
    Rect_t area = {.x = x, .y = y, .width = sunrise_width, .height = sunrise_height};
    epd_draw_grayscale_image(area, (uint8_t *)sunrise_data);
}

void DrawSunsetImage(int x, int y) {
    Rect_t area = {.x = x, .y = y, .width = sunset_width, .height = sunset_height};
    epd_draw_grayscale_image(area, (uint8_t *)sunset_data);
}

void DrawUVI(int x, int y) {
    Rect_t area = {.x = x, .y = y, .width = uvi_width, .height = uvi_height};
    epd_draw_grayscale_image(area, (uint8_t *)uvi_data);
}

void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title,
               float DataArray[], int readings, bool auto_scale, bool barchart_mode) {
    constexpr float auto_scale_margin = 0.00;
    constexpr int y_minor_axis = 5;
    constexpr int number_of_dashes = 20;

    setFont(OpenSans10B);
    int maxYscale = -10000;
    int minYscale = 10000;
    int last_x, last_y;
    float x2, y2;
    if(auto_scale == true) {
        for(int i = 1; i < readings; i++) {
            if(DataArray[i] >= maxYscale)
                maxYscale = DataArray[i];
            if(DataArray[i] <= minYscale)
                minYscale = DataArray[i];
        }
        maxYscale = round(maxYscale + auto_scale_margin);
        Y1Max = round(maxYscale + 0.5);
        if(minYscale != 0)
            minYscale = round(minYscale - auto_scale_margin);
        Y1Min = round(minYscale);
    }

    last_x = x_pos + 1;
    last_y = y_pos + (Y1Max - constrain(DataArray[1], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight;
    drawRect(x_pos, y_pos, gwidth + 3, gheight + 2, Color::Grey);
    drawString(x_pos - 20 + gwidth / 2, y_pos - 28, title, Alignment::CENTER);
    for(int gx = 0; gx < readings; gx++) {
        x2 = x_pos + gx * gwidth / (readings - 1) - 1;
        y2 = y_pos + (Y1Max - constrain(DataArray[gx], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight + 1;
        if(barchart_mode) {
            fillRect(last_x + 2, y2, (gwidth / readings) - 1, y_pos + gheight - y2 + 2, Color::Grey);
        } else {
            drawLine(last_x, last_y - 1, x2, y2 - 1, Color::Grey);
            drawLine(last_x, last_y, x2, y2, Color::Grey);
        }
        last_x = x2;
        last_y = y2;
    }

    for(int spacing = 0; spacing <= y_minor_axis; spacing++) {
        for(int j = 0; j < number_of_dashes; j++) {
            if(spacing < y_minor_axis)
                drawFastHLine((x_pos + 3 + j * gwidth / number_of_dashes),
                              y_pos + (gheight * spacing / y_minor_axis), gwidth / (2 * number_of_dashes),
                              Color::Grey);
        }
        if((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing) < 5 || title == TXT_PRESSURE_IN) {
            drawString(x_pos - 10, y_pos + gheight * spacing / y_minor_axis - 5,
                       String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1),
                       Alignment::RIGHT);
        } else {
            if(Y1Min < 1 && Y1Max < 10) {
                drawString(x_pos - 3, y_pos + gheight * spacing / y_minor_axis - 5,
                           String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1),
                           Alignment::RIGHT);
            } else {
                drawString(x_pos - 7, y_pos + gheight * spacing / y_minor_axis - 5,
                           String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 0),
                           Alignment::RIGHT);
            }
        }
    }
    for(int i = 0; i < 3; i++) {
        drawString(20 + x_pos + gwidth / 3 * i, y_pos + gheight + 10, String(i) + "d", Alignment::LEFT);
        if(i < 2)
            drawFastVLine(x_pos + gwidth / 3 * i + gwidth / 3, y_pos, gheight, Color::LightGrey);
    }
}

void drawString(int x, int y, String text, Alignment align) {
    char *data = const_cast<char *>(text.c_str());
    int x1, y1;
    int w, h;
    int xx = x, yy = y;
    get_text_bounds(&currentFont, data, &xx, &yy, &x1, &y1, &w, &h, NULL);
    if(align == Alignment::RIGHT)
        x = x - w;
    if(align == Alignment::CENTER)
        x = x - w / 2;
    int cursor_y = y + h;
    write_string(&currentFont, data, &x, &cursor_y, framebuffer);
}

void fillCircle(int x, int y, int r, Color color) {
    epd_fill_circle(x, y, r, to_underlying(color), framebuffer);
}

void drawFastHLine(int16_t x0, int16_t y0, int length, Color color) {
    epd_draw_hline(x0, y0, length, to_underlying(color), framebuffer);
}

void drawFastVLine(int16_t x0, int16_t y0, int length, Color color) {
    epd_draw_vline(x0, y0, length, to_underlying(color), framebuffer);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color) {
    epd_write_line(x0, y0, x1, y1, to_underlying(color), framebuffer);
}

void drawCircle(int x0, int y0, int r, Color color) {
    epd_draw_circle(x0, y0, r, to_underlying(color), framebuffer);
}

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, Color color) {
    epd_draw_rect(x, y, w, h, to_underlying(color), framebuffer);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, Color color) {
    epd_fill_rect(x, y, w, h, to_underlying(color), framebuffer);
}

void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, Color color) {
    epd_fill_triangle(x0, y0, x1, y1, x2, y2, to_underlying(color), framebuffer);
}

void drawPixel(int x, int y, Color color) { epd_draw_pixel(x, y, to_underlying(color), framebuffer); }

void setFont(GFXfont const &font) { currentFont = font; }

void edp_update() { epd_draw_grayscale_image(epd_full_screen(), framebuffer); }