
#ifndef CONFIG_HANDLER_H
#define CONFIG_HANDLER_H

#include "CardReader.h"

struct Config
{
  // last time the config was saved
  unsigned long time;

  // state of tracker
  int state;

  // initialize the ssid as tracker-01
  char ssid[11] = "tracker-01";
};

class ConfigHandler
{
public:
  ConfigHandler(CardHandler cardHandler);
  void init();
  void saveConfig();
  Config config;

private:
  CardHandler _cardHandler;
};

#endif // CARD_READER_H
