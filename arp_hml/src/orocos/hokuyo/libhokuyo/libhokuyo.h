#pragma once

#include "sys/winHeaders.h"

#include <stdexcept>
//#include <termios.h>
//#include <string>
#include <vector>

#if defined(_MSC_VER) && _MSC_VER < 1600
  typedef __int8 int8_t;
  typedef unsigned __int8 uint8_t;
  typedef __int16 int16_t;
  typedef unsigned __int16 uint16_t;
  typedef __int32 int32_t;
  typedef unsigned __int32 uint32_t;
  typedef __int64 int64_t;
  typedef unsigned __int64 uint64_t;
#else
  #include <stdint.h>
#endif


namespace hokuyo
{
  const uint32_t MAX_READINGS = 1128;

  const int MAX_CMD_LEN = 100;

  const int MAX_SKIPPED = 1000000;
  
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }
  
  DEF_EXCEPTION(Exception, std::runtime_error);

  DEF_EXCEPTION(TimeoutException, Exception);

  DEF_EXCEPTION(CorruptedDataException, Exception);

  DEF_EXCEPTION(RepeatedTimeException, Exception);

#undef DEF_EXCEPTION

  struct LaserConfig
  {
    float min_angle;
    float max_angle;
    float ang_increment;
    float time_increment;
    float scan_time;
    float min_range;
    float max_range;
    float range_res;
  };


  struct LaserScan
  {
    std::vector<float> ranges;
    std::vector<float> intensities;
    uint64_t self_time_stamp;
    uint64_t system_time_stamp;
    LaserConfig config;
  };

  
  class Laser
  {
  public:
    Laser();

    ~Laser();
  

    void open(const char * port_name);


    void close();


    void reset();
  
    //bool portOpen() {  return laser_fd_ != -1; }
    bool portOpen() {  return hCom_ != INVALID_HANDLE_VALUE; }

        // sets up model 04LX to work in SCIP 2.0 mode
    void setToSCIP2();

    int sendCmd(const char* cmd, int nbMaxAttempts = -1);


    void getConfig(LaserConfig& config);



    int pollScan(LaserScan& scan, double min_ang, double max_ang, int clustering = 0, int timeout = -1);


    int requestScans(bool intensity, double min_ang, double max_ang, int clustering = 0, int skip = 0, int num = 0);


    int serviceScan(LaserScan& scan);


    int laserOff();


    int laserOn();


    int stopScanning();


    std::string getID();


    std::string getStatus();


    long long calcLatency(bool intensity, double min_ang, double max_ang, int clustering = 0, int skip = 0, int num = 0);

    void clearLatency()
    {
      offset_ = 0;
    }

    long long getLatency()
    {
      return offset_;
    }


    std::string getFirmwareVersion();


    std::string getVendorName();


    std::string getProductName();


    std::string getProtocolVersion();


    bool isIntensitySupported();

    void queryVersionInformation();
  
    int laserWrite(const char* msg);

    int laserReadline(char *buf, int len, int nbMaxAttempts = -1);

    int laserFlush();

    uint64_t readTime(int nbMaxAttempts = -1);

  private:
    //the Hokuyo clock. Returns the median of reps measurements.
    long long int getHokuyoClockOffset(int reps);

    // timestamp (PC) and the scan timestamp (Hokuyo).
    long long int getHokuyoScanStampToSystemStampOffset(bool intensity, double min_ang, double max_ang, int clustering, int skip, int reps);
     
    void querySensorConfig();

    char* laserReadlineAfter(char *buf, int len, const char *str, int nbMaxAttempts = -1);

    bool checkSum(const char* buf, int buf_len);

    void readData(LaserScan& scan, bool has_intensity, int nbMaxAttempts = -1);

    int dmin_;
    int dmax_;
    int ares_;
    int amin_;
    int amax_;
    int afrt_;
    int rate_;

    int wrapped_;

    unsigned int last_time_;

    unsigned int time_repeat_count_;

    long long offset_;

    //int laser_fd_;

    std::string vendor_name_;
    std::string product_name_;
    std::string serial_number_;
    std::string protocol_version_;
    std::string firmware_version_;

    char read_buf[256];
    int read_buf_start;
    int read_buf_end;

  private:
    std::string error_message_;
    HANDLE hCom_;
    int current_timeout_;
    std::string com_name_;

  };

}
