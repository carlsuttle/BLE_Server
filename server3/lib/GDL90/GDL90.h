#ifndef _GDL90_
#define _GDL90_

#include <arduino.h>

#define GDL90_MAX_MESSAGE_SIZE 200

//https://www.faa.gov/sites/faa.gov/files/air_traffic/technology/adsb/archival/GDL90_Public_ICD_RevA.PDF
//https://www.foreflight.com/connect/spec/
class GDL90 {
  private:  
    typedef union
    {
      short numberShort;
      uint8_t bytes[2];
    } TWOBYTEUNION_t;
    
    unsigned int CRC16Table[256];
    byte scratchBuffer[GDL90_MAX_MESSAGE_SIZE];


    void crcCompute(byte *block, int len, byte *crcByteArray);
    void crcInit();
    void UnEscape(byte *buffer,int *len);
    void Escape(byte *buffer,int *len); 
    void makeLatLng(float v,byte *buffer);
    bool unPackMessage(byte *buffer,int *len);     
    void buildFinalMessage(byte *buffer,int *len);
    
  public:
    GDL90();

    //methods AHRS->client
    void makeHeartbeat(byte *msg,String gpsUTC,int *len);
    void makeOwnshipGeometricAltitudeReport(int altitude,byte *msg,int *len);
    void makeAHRSMessage(float heading, float pitch, float roll, byte *msg,int *len);
    void makeOwnershipMessage(float lat, float lon, float vx, float vsi, float heading, byte *msg, int *len);

    //methods client->AHRS
    bool decodeAHRSMessage(float *heading, float *pitch, float *roll, byte *msg,int *len);
    
    //testing methods
    void printBufferInHEX(byte *buffer,int len); 
    void regressionTest();

};
#endif //_GDL90_
