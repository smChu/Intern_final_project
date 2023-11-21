#include "../inc/test.h"

Test::Test(Serial *s, bool *serialRead, mutex *m, queue<uint8_t> *readByte)
{
  this->s = s;
  this->serialRead = serialRead;
  this->m = m;
  this->readByte = readByte;

  conversationStatus = CONVERSATION_REQUEST;
}

Test::~Test()
{

}

void Test::myCallBack(const geometry_msgs::TwistConstPtr& msg)
{
  _x = msg->linear.x;
  _z = msg->angular.z;
}

void Test::Algorithm_Test()
{
  switch (conversationStatus)
  {
  case CONVERSATION_REQUEST:
  {
    try
    {
      vector<uint8_t> packet;

      stmPacketGenerator.writePacket(packet);
      
      s->write(packet);
    }
    catch (serial::IOException e)
    {
      cerr << "Port open failed." << e.what() << endl;
      s->close();
    }

    conversationStatus = CONVERSATION_RESPONSE;

    break;
  }
  case CONVERSATION_RESPONSE:
  {
    if(!readByte->empty())
    {
      while(!readByte->empty())
      {
        int readBytefront = (int)readByte->front();

        stmPacketTranslator.readPacket(readBytefront);
        
        m->lock();
        readByte->pop();
        m->unlock();
      }
    }

    conversationStatus = CONVERSATION_REQUEST;

    break;
  }
  }
}
