#ifndef STM_WRITE_H
#define STM_WRITE_H

#include "header.h"

class StmPacketGenerator
{
private:

public:
  StmPacketGenerator();
  ~StmPacketGenerator();
  // 11/22
  int _x;
  int _z;
  int ID_1=1;
  int ID_2=2;
  int ID_3=3;
  int ID_5=4;
  int ID_6=5;
  // 11/22

  void divideByte(vector<uint8_t> &packet, int value, int length);
  void divideFloat(vector<uint8_t> &packet, float value);
  void update_header(vector<uint8_t> &packet);
  void update_data(vector<uint8_t> &packet);
  void update_crc(vector<uint8_t> &data_blk_ptr);
  void writePacket(vector<uint8_t> &packet);
};

#endif // STM_WRITE_H
