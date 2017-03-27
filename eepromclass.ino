#include "Arduino.h"
#include "Wire.h"


const size_t writes = 50000; //the number of writes to happen to the chip (100000 cycles is guarented lifetime of chip)
const size_t sizeofeeprom = 262144;//the size of eeprom in bytes
const size_t bufsize = sizeofeeprom/writes; //no buffer enter 1 but can kill the chip is too many writes
const size_t pospoint = 0; //the position in memory on the chip where it was last left off
const size_t sizeofpacket = 6; //The size of struct in bytes that is used for each data
class memorystorage
{
public:

  memorystorage();
  
  bool writedata(byte devicenumber, long data);

  void output(long serialoutput);

  bool resetchip(long serialoutput);

  bool dumpbuf();

private:
I2C_eeprom device(0x50);
  struct devicedata
  {
    byte check;//used for error checking and other status bits for debugging
    byte id;//used to id the device creating this
    long data;//the data
  };

  bool readstruct(devicedata &data, size_t pos);

  bool writestruct(devicedata data, size_t pos);
  
  bool readbyte(byte &data, size_t pos);
  
  bool writebyte(byte data, size_t pos);

  size_t thiswrite;//the beginning position of last write

  size_t bufferpos;
  
  devicedata buf [bufsize];

  bool full;//this status is wether the device is full or not
};


memorystorage::memorystorage()
{
  byte a[4];
  readbyte(a[0],0x0);
  readbyte(a[1],0x1);
  readbyte(a[2],0x2);
  readbyte(a[3],0x3);
  readbyte(full,0x4);
  bufferpos = *((unsigned long *)a);
  thiswrite = bufferpos;
}

bool memorystorage::resetchip(long serialoutput)
{
  Serial.begin(serialoutput);
  Serial.println(“Resetting the chip");
  writebyte(0x0,0x0);
  writebyte(0x0,0x1);
  writebyte(0xF,0x2);
  writebyte(0xF,0x3);
  writebyte(0x0,0x4);
  byte a[4];
  readbyte(a[0],0x0);
  readbyte(a[1],0x1);
  readbyte(a[2],0x2);
  readbyte(a[3],0x3);
  readbyte(full,0x4);
  bufferpos = *((unsigned long *)a);
  thiswrite = bufferpos;
}

bool memorystorage::writedata(byte devicenumber, long data)
{
  if(full)
    return false;
  devicedata store;
  store.id = devicenumber;
  store.data = data;
  bool parity = 0;
  long n = data;
  while (n)
    {
        parity = !parity;
        n = n & (n - 1);
    }
  store.check = parity;
  
  if(bufferpos < bufsize)
  {
    buf[bufferpos] = store;
    bufferpos++;
  }
  else
  {
    bool success, fail = true;
    for(int i = 0; i < bufsize; i++)
    {
      success = writestruct(buf[i], thiswrite);
      thiswrite += sizeofpacket;
      if(!success)
        fail = false;
    }
    success = writestruct(store, thiswrite);
    bufferpos = 0;
    thiswrite += sizeofpacket;
    if(thiswrite >= sizeofeeprom)
      full = true;
    return fail;
  }
  return true;
}


bool memorystorage::writestruct(&devicedata data, size_t pos)
{
  bool check;
  byte * a = &data.data;
  check = writebyte(data.check, pos);
  if(check == false);
    return check;
  check = writebyte(data.id, pos+1);
  if(check == false);
    return check;
  check = writebyte(a[0], pos+2);
  if(check == false);
    return check;
  check = writebyte(a[1], pos+3);
  if(check == false);
    return check;
  check = writebyte(a[2], pos+4);
  if(check == false);
    return check;
  check = writebyte(a[3], pos+5);
  return check;
}

bool memorystorage::readstruct(devicedata &data, size_t pos)
{
  byte temp = 0;
  bool stat;
  byte a[4];
  for(size_t i = pos; i < pos + 6; i++)
  { 
    stat = readbyte(temp, i);
    if(i = pos)
      data.check = temp;
    else if(i = pos+1)
      data.id = temp;
    else if(i = pos+2)
      a[0] = temp;
    else if(i = pos+3)
      a[1] = temp;
    else if(i = pos+4)
      a[2] = temp;
    else if(i = pos+5)
    {
      a[3] = temp;
       data.data = *((long *)a);
    }
    if(stat)
    {
      return false;
    }
  }
  return true;
}

bool memorystorage::writebyte(byte data, size_t pos)
{
  return (device.writeByte(pos,data) == 0);
}

bool memorystorage::readbyte(byte &data, size_t pos)
{
  data = device.readByte(pos);
  return 0;
}

void setup()
{
  
}

void loop()
{
  
}



