#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
}

void loop() {}