#include "BluetoothHandler.h"

BluetoothHandler::BluetoothHandler() : _bluetoothSerial()
{
  _connected = false;
}

void BluetoothHandler::start()
{
  _bluetoothSerial.begin("ESP32test"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  delay(5000);
  _connected = true;
}

void BluetoothHandler::stop()
{
  _bluetoothSerial.println("AT+BTPAN=0,0");
  delay(5000);
  _connected = false;
}

bool BluetoothHandler::isConnected()
{
  return _connected;
}

String BluetoothHandler::httpGet(String url)
{
  String result = "";

  if (_connected)
  {
    _bluetoothSerial.println("AT+HTTPPARA=\"CID\",1");
    _bluetoothSerial.println("AT+HTTPPARA=\"URL\",\"" + url + "\"");
    _bluetoothSerial.println("AT+HTTPACTION=0");
    delay(10000);

    while (_bluetoothSerial.available())
    {
      result += (char)_bluetoothSerial.read();
    }
  }

  return result;
}

void BluetoothHandler::syncEspClock()
{
  if (_connected)
  {
    _bluetoothSerial.println("AT+CIPSNTPCFG=1,8,\"pool.ntp.org\",1");
    _bluetoothSerial.println("AT+CIPSNTP=1");
    delay(10000);
  }
}
