#include <GDL90.h>


GDL90::GDL90() {
  crcInit();
}

void GDL90::crcCompute(byte *block, int len, byte *crcByteArray)   
{
  unsigned long int i;     
  unsigned int crc = 0;     
  for (i = 0; i < len; i++){   
    if(crc>>8 > 256 || crc>>8 < 0) {
      //Serial.println(crc>>8);            
    } else {
      crc = CRC16Table[crc >> 8] ^ (crc << 8)&0xffff ^ block[i];
    }
  }     
  crcByteArray[0]=crc & 0x00ff;
  crcByteArray[1]=(crc & 0xff00) >> 8;
}

void GDL90::crcInit() {    
  unsigned int i, bitctr, crc;     
  for (i = 0; i < 256; i++) {         
    crc = (i << 8);         
    for (bitctr = 0; bitctr < 8; bitctr++) {             
      crc = ((crc << 1) & 0xffff) ^ ((crc & 0x8000) ? 0x1021 : 0);
      }         
      CRC16Table[i] = crc;
  } 
}

void GDL90::printBufferInHEX(byte *buffer,int len) {
  // for(int i=0;i<len;i++) {
  //   byte b=buffer[i];
  //   Serial.print(b>>4,  HEX);
  //   Serial.print(b&0x0F,HEX);
  // }
  // Serial.println("");
}

//The unescape process remove the byte stuffing and hence the returned buffer may have less chars..use len on return to determine number of chars in buffer
void GDL90::UnEscape(byte *buffer,int *len) {
  int ii=0;
  for(int i=0;i<*len;i++) {
    if(buffer[i]==0x7D) {
      buffer[ii]=buffer[i+1]^0x020;
      i++;
    } else {
      buffer[ii]=buffer[i];
    }
    ii++;
  }
  *len=ii;
}

//The escape may add bytes to buffer due to byte stuffing..ensure buffer is big enough to handle the addition chars..use len on return to determine the new len of buffer
void GDL90::Escape(byte *buffer,int *len) {
  int ii=0;
  byte escapedBuffer[GDL90_MAX_MESSAGE_SIZE];
  for(int i=0;i<*len;i++) {
    if(buffer[i]==0x7D||buffer[i]==0x7E) {
      escapedBuffer[ii]=0x7D;
      ii++;
      escapedBuffer[ii]=buffer[i]^0x020;
    } else {
      escapedBuffer[ii]=buffer[i];
    }
    ii++;
  }
  memcpy(buffer,escapedBuffer,ii);
  *len=ii;
}

void GDL90::makeHeartbeat(byte *msg,String gpsUTC,int *len)  {
  msg[0] = 0x00; // Message type "Heartbeat".
  msg[1] = 0x01; // "UAT Initialized".
  msg[1] = msg[1] | 0x80;
  msg[1] = msg[1] | 0x10 ;

  //240422050127.60 18087
  int secondsSinceMidnightUTC=atoi(gpsUTC.substring(6,8).c_str())*3600+atoi(gpsUTC.substring(8,10).c_str())*60+atoi(gpsUTC.substring(10,12).c_str());

  //Serial.println(gpsUTC+" "+secondsSinceMidnightUTC);

  msg[2] = byte(((secondsSinceMidnightUTC >> 16) << 7) | 0x1); // UTC OK.
  msg[3] = byte((secondsSinceMidnightUTC & 0xFF));
  msg[4] = byte((secondsSinceMidnightUTC & 0xFFFF) >> 8);

  *len=7;
  buildFinalMessage(msg,len);
}

void GDL90::makeOwnshipGeometricAltitudeReport(int altitude,byte *msg,int *len) {
  msg[0] = 0x0B;                                 // Message type "Ownship Geo Alt".
  int alt = altitude/5;                              // GPS Altitude, encoded to 16-bit int using 5-foot resolution
  msg[1] = byte(alt >> 8);                       // Altitude.
  msg[2] = byte(alt & 0x00FF);                   // Altitude.

  //TODO: "Figure of Merit". 0x7FFF "Not available".
  msg[3] = 0x00;
  msg[4] = 0x0A;
  
  *len=5;
  buildFinalMessage(msg,len);
}

void GDL90::buildFinalMessage(byte *buffer,int *len) {
  byte crc[2];
  memcpy(scratchBuffer,buffer,*len);
  crcCompute(scratchBuffer,*len,crc);
  memcpy(&scratchBuffer[*len],crc,2);
  *len+=2;
  
  Escape(scratchBuffer,len);
  buffer[0]=0x7E;
  memcpy(&buffer[1],scratchBuffer,*len);
  buffer[*len+1]=0x7E;
  *len+=2;
}

bool GDL90::unPackMessage(byte *buffer,int *len) {
  bool valid=false;
  byte crc[2];
  
  memcpy(scratchBuffer,&buffer[1],*len-2); //strip the flag bytes 0x7E
  *len-=2;

  UnEscape(scratchBuffer,len);
  
  crcCompute(scratchBuffer,*len-2,crc);

  if(scratchBuffer[*len-2]==crc[0] && scratchBuffer[*len-1]==crc[1]) {
    valid=true;
    memcpy(buffer,scratchBuffer,*len-2);
    *len-=2; //reduce for the crc
  } else {
    valid=false;
  }

  return valid;
}

//https://www.foreflight.com/connect/spec/
void GDL90::makeAHRSMessage(float heading, float pitch, float roll, byte *msg, int *len) {
  msg[0] = 0x65;                                
  msg[1] = 0x01;

  roll*=10;
  msg[2] = byte((short)roll >> 8);                       
  msg[3] = byte((short)roll & 0x00FF);    

  pitch*=10;
  msg[4] = byte((short)pitch >> 8);                       
  msg[5] = byte((short)pitch & 0x00FF);    

  heading*=10;
  msg[6] = byte((short)heading >> 8);                       
  msg[7] = byte((short)heading & 0x00FF);
  msg[7] = msg[7] | 0x80; //true heading 0x00 mag heading 0x80, the most significant bit indicates 1-magnetic 0-true    

  //IAS
  msg[8]=0xFF;
  msg[9]=0xFF;
  
  //TAS
  msg[10]=0xFF;
  msg[11]=0xFF;
                
  *len=12;
  buildFinalMessage(msg,len);
}

//If messages is invalid the heading, pitch and roll passed into the method remain unchanged
bool GDL90::decodeAHRSMessage(float *heading, float *pitch, float *roll, byte *msg,int *len) {
  bool valid=unPackMessage(msg,len);

  if(valid) {
    TWOBYTEUNION_t b;
    
    b.bytes[0]=msg[3];
    b.bytes[1]=msg[2];
    *roll=(float)b.numberShort/10.0;
    
    b.bytes[0]=msg[5];
    b.bytes[1]=msg[4];
    *pitch=(float)b.numberShort/10.0;
  
    b.bytes[0]=msg[7]&(~0x80);      //the most significant bit indicates 1-magnetic 0-true
    b.bytes[1]=msg[6];
    *heading=(float)b.numberShort/10.0;
  }
  return valid;
}

void GDL90::makeLatLng(float v,byte *buffer){
  v = v / float(180.0 / 8388608.0);
  int wk = int(v);

  buffer[0] = byte((wk & 0xFF0000) >> 16);
  buffer[1] = byte((wk & 0x00FF00) >> 8);
  buffer[2] = byte((wk & 0x0000FF));
}

void GDL90::makeOwnershipMessage(float lat, float lon, float vx, float vsi, float heading, byte *msg, int *len) {
    //the following is sample ownership message from the gdl90 spec, for now I am replacing part of the message with our data to cut corners
    byte dummyOwnership2[] ={0x0A,0x07,0x00,0xB0,0xAB,0x01,0x45,0x20,0x49,0x01,0x1F,0x4E,0xEF,0x38,0x15,0x32,0xA8,0x35,0x89,0x56,0x78,0x2,0x0F,0x20,0x09,0x20,0xA9,0x00};
    memcpy(msg,dummyOwnership2,28);

    byte latBuffer[3];
    makeLatLng(lat,latBuffer);
    byte LonBuffer[3];
    makeLatLng(lon,LonBuffer);
    memcpy(&msg[5],latBuffer,3);
    memcpy(&msg[8],LonBuffer,3);

    short gdSpeed=vx*1.94384; // 1kt resolution.
    msg[14] = byte((gdSpeed & 0xFF0) >> 4);
    msg[15] = byte((gdSpeed & 0x00F) << 4);

    short verticalVelocity=vsi/64; 
    msg[15] = msg[15] | byte((verticalVelocity&0x0F00)>>8);
    msg[16] = byte(verticalVelocity & 0xFF);

    float tempTrack=heading + float(360.0 / 256.0)/2; // offset by half the 8-bit resolution to minimize binning error
    if(tempTrack > 360) {
      tempTrack -= 360;
    }
    if(tempTrack < 0) {
      tempTrack += 360;
    }
    byte trk = byte(tempTrack / float(360.0 / 256.0)); // Resolution is ~1.4 degrees.
    msg[17] = byte(trk);

    *len=28;
    buildFinalMessage(msg,len);
}
    

void GDL90::regressionTest() {    
  byte buffer[GDL90_MAX_MESSAGE_SIZE];
  byte crc[2];

  
  Serial.println("Escape");
  Serial.println("Expected: 7D5D7D5E");
  Serial.print(  "Actual:   "); 
  int len=2;
  buffer[0]=0x7D;
  buffer[1]=0x7E;
  Escape(buffer,&len);
  printBufferInHEX(buffer,len);

  Serial.println("UnEscape");
  Serial.println("Expected: 7D7E");
  Serial.print(  "Actual:   ");
  UnEscape(buffer,&len);
  printBufferInHEX(buffer,len);

  Serial.println("UnEscape 2");
  Serial.println("Expected: 7E6501FF7EFF7E0080FFFFFFFFD91F7E");
  Serial.print(  "Actual:   ");
  byte test0[] = {0x7E,0x65,0x01,0xFF,0x7D,0x5E,0xFF,0x7D,0x5E,0x00,0x80,0xFF,0xFF,0xFF,0xFF,0xD9,0x1F,0x7E};
  memcpy(buffer,test0,sizeof(test0));
  len=sizeof(test0);
  UnEscape(buffer,&len);
  printBufferInHEX(buffer,len);

  Serial.println("checksum1");
  Serial.println("Expected: B38B");
  Serial.print(  "Actual:   ");
  byte test1[] = {0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02};
  memcpy(buffer,test1,sizeof(test1));
  len=sizeof(test1);
  UnEscape(buffer,&len);
  crcCompute(buffer,len,crc);
  printBufferInHEX(crc,2);
    
  Serial.println("checksum2");
  Serial.println("Expected: 9733");
  Serial.print(  "Actual:   ");
  byte test2[] = {0x0a, 0x00, 0x00, 0x00, 0x00, 0x15, 0x76, 0x78, 0xba, 0x8d, 0x1f, 0x03, 0xb9, 0x88, 0x00, 0x00, 0x00, 0xa8, 0x01, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00};
  memcpy(buffer,test2,sizeof(test2));
  len=sizeof(test2);
  UnEscape(buffer,&len);
  crcCompute(buffer,len,crc);
  printBufferInHEX(crc,2);

  // Serial.println("Final message1");
  // Serial.println("Expected: 7E008141DBD00802B38B7E");
  // Serial.print(  "Actual:   ");
  byte test3[] = {0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02};
  memcpy(buffer,test3,sizeof(test3));
  len=sizeof(test3);
  buildFinalMessage(buffer,&len);
  printBufferInHEX(buffer,len);

  // Serial.println("Final message2");
  // Serial.println("Expected: 7E0A00000000157678BA8D1F03B988000000A80120202020202020200097337E");
  // Serial.print(  "Actual:   ");
  byte test4[] = {0x0a, 0x00, 0x00, 0x00, 0x00, 0x15, 0x76, 0x78, 0xba, 0x8d, 0x1f, 0x03, 0xb9, 0x88, 0x00, 0x00, 0x00, 0xa8, 0x01, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00};
  memcpy(buffer,test4,sizeof(test4));
  len=sizeof(test4);
  buildFinalMessage(buffer,&len);
  printBufferInHEX(buffer,len);

  // Serial.println("Unpack message2");
  // Serial.println("Expected: 0A00000000157678BA8D1F03B988000000A801202020202020202000");
  // Serial.print(  "Actual:   ");
  bool valid=unPackMessage(buffer,&len);
  printBufferInHEX(buffer,len);
  // Serial.println("Expected: true");
  // Serial.print(  "Actual:   ");
  // Serial.println(valid?"true":"false");  

  // Serial.println("Final makeHeartbeat 1");
  // Serial.println("Expected: 7E009101A7461576863E7E");
  // Serial.print(  "Actual:   ");
  makeHeartbeat(buffer,"240422050127.60",&len);
  printBufferInHEX(buffer,len);

  //Serial.println("AHRS message 1");
  float heading,pitch,roll;
  makeAHRSMessage(359.9, 9.1, 32.2, buffer, &len);
  decodeAHRSMessage(&heading,&pitch,&roll,buffer, &len);
  // Serial.println("Expected:  32.20");
  // Serial.print(  "Actual:    ");
  // Serial.println(roll);  
  // Serial.println("Expected:  9.10");
  // Serial.print(  "Actual:    ");
  // Serial.println(pitch);
  // Serial.println("Expected:  359.90");
  // Serial.print(  "Actual:    ");
  // Serial.println(heading);

}
