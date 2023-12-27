#ifndef TEST_H
#define TEST_H

#include "header.h"
#include "stm_read.h"
#include "stm_write.h"
#include <tutorial_msgs/mydmxel.h>
#include <std_msgs/Int32.h>



class Test
{
private:
  StmPacketGenerator stmPacketGenerator;
  StmPacketTranslator stmPacketTranslator;
  
  enum ConversationStatus
  {
    CONVERSATION_REQUEST,
    CONVERSATION_RESPONSE
  };
  ConversationStatus conversationStatus;

public:
  Test(Serial *s, bool *serialRead, mutex *m, queue<uint8_t> *readByte);
  ~Test();

  //float _x, _z;
  int mode_flag = 1;

  void myCallBack(const tutorial_msgs::mydmxelConstPtr& msg);
  void myCallBack_autorace(const tutorial_msgs::mydmxelConstPtr& msg);
  void myCallBack_mode(const std_msgs::Int32::ConstPtr& msg);

  void Algorithm_Test();

  Serial *s;
  bool *serialRead;
  mutex *m;
  queue<uint8_t> *readByte;
};

#endif // MINIBOT_H
