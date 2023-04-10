#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <BluetoothSerial.h>

class BluetoothHandler
{
public:
  BluetoothHandler();
  void start();
  void stop();
  void syncEspClock();
  bool isConnected();
  String httpGet(String url);

private:
  BluetoothSerial _bluetoothSerial;
  bool _connected;
};

#endif // BLUETOOTH_HANDLER_H
