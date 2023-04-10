
#ifndef CARD_READER_H
#define CARD_READER_H

struct Config
{
  String ssid;
  String time;
} config;

class ConfigHandler
{
public:
  ConfigHandle(CardHandler cardHandler);
  void init();

private:
  CardHandler _cardHandler;
};

#endif // CARD_READER_H
