#ifndef STM_WRITE_H
#define STM_WRITE_H

#include "header.h"

class StmPacketGenerator
{
private:

public:
  StmPacketGenerator();
  ~StmPacketGenerator();

  void divideByte(vector<uint8_t> &packet, int value, int length);
  void update_header(vector<uint8_t> &packet);
  void update_data(vector<uint8_t> &packet);
  void update_crc(vector<uint8_t> &data_blk_ptr);
  void writePacket(vector<uint8_t> &packet);
};

#endif // STM_WRITE_H
