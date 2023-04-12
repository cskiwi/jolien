
#include <Arduino.h>
#include "ConfigHandler.h"
#include "CardReader.h"
#include "SdFat.h"

//  save configuration to a config.txt file
char configFileName[12] = "/config.txt";

#define error(s)     \
  Serial.println(s); \
  while (1)          \
  {                  \
  }

// Constructor
ConfigHandler::ConfigHandler(CardHandler cardHandler)
{
  this->_cardHandler = cardHandler;
}

void ConfigHandler::init()
{
  file_t configFile = this->_cardHandler.getFile(configFileName, O_READ);

  Serial.println("Reading config file");

  int counter = 0;

  // Read the contents of the file
  while (configFile.available() && counter < 10)
  {
    counter++;
    String line = configFile.readStringUntil('\r\n');

    // trim the line
    line.trim();

    if (line.startsWith("ssid:"))
    {
      String ssid = line.substring(5);
      ssid.toCharArray(config.ssid, 32);
    }
    else if (line.startsWith("time:"))
    {
      long time = line.substring(5).toInt();
      config.time = time;
    }

    Serial.println(line);
  }

  // Print the config
  Serial.printf("ssid:%s\r\n", config.ssid);
  Serial.printf("time:%ld\r\n", config.time);

  // Close the file
  configFile.close();
}

void ConfigHandler::saveConfig()
{
  Serial.println("Saving config file");
  file_t configFile = this->_cardHandler.getFile(configFileName, O_WRITE | O_CREAT | O_TRUNC);

  Serial.printf("ssid:%s\r\n", config.ssid);
  Serial.printf("time:%ld\r\n", config.time);

  // Write ssid and time to the file
  configFile.printf("ssid:%s\r\n", config.ssid);
  configFile.printf("time:%ld\r\n", config.time);

  // Save the new config to the file
  configFile.close();
  Serial.println(F("Done"));
}
