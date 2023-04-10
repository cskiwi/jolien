
#include <Arduino.h>
#include "ConfigHandler.h"
#include "CardReader.h"

ConfigHandler::ConfigHandler(CardHandler cardHandler)
{
  _cardHandler = cardHandler;
}
