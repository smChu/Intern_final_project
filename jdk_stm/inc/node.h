#ifndef NODE_H
#define NODE_H

#include "header.h"
#include "test.h"

#define baudrate 115200 //manipulator = 1000000
#define rosrate 80 //manipulator = 200
#define startdelay 0 //manipulator = 100000000
#define readbuffersize 256 //manipulator = 1024

class Node
{
public:
  Node(int &argc, char **argv);
  ~Node();
  void run();

  float _x, _z;

private:
  string portname = "ttyUSB0";

  Test test;

  ros::Subscriber sub;

  bool retry();
  void timerCallback();
  static void serialReadThread();

  static Serial s;
  static bool serialRead;
  static mutex m;
  static queue<uint8_t> readByte;
};

#endif // NODE_H
