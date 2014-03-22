/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <string>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

//#include <stdio.h>
//#include <string.h>
//#include <errno.h>
//#include <termios.h>
//#include <math.h>
//#include <poll.h>
//#include <signal.h>

#include <algorithm>
#include <cstdio>

#include "libhokuyo.h"
#include "sys/timer.h"

#include <rtt/Logger.hpp>
using namespace RTT;

//#include <time.h>

//#include <fcntl.h>
//#include <io.h>

//! Macro for throwing an exception with a message, passing args
#define HOKUYO_EXCEPT(except, msg, ...) \
  { \
    char buf[1000]; \
    sprintf(buf, msg " (in %s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
  }


//! Helper function for querying the system time
static uint64_t timeHelper()
{
  return (uint64_t) (xde::sys::Timer::GetAbsoluteTime() * 1000000000);  //time in ns
}


hokuyo::Laser::Laser() :
                      dmin_(0), dmax_(0), ares_(0), amin_(0), amax_(0), afrt_(0), rate_(0),
                      wrapped_(0), last_time_(0), time_repeat_count_(0), offset_(0)
                      , hCom_(INVALID_HANDLE_VALUE)
                      //, laser_fd_(-1)
{ 
  
}


hokuyo::Laser::~Laser ()
{
  if (portOpen())
    close();
}


void hokuyo::Laser::open(const char * port_name)
{
  //Logger::In inLogger("hokuyo::Laser::open");

  //log( Debug ) << "###################################################################" << endlog(); 
  //log( Debug ) << "Begin opening device..." << endlog(); 
  //log( Debug ) << "###################################################################" << endlog(); 

  if (portOpen())
  {
    //log( Debug ) << "Already opened : we first close before reopening" << endlog(); 
    close();
  }

  try
  {
    enum { NameLength = 11 };
    char adjusted_device[NameLength];
    sprintf(adjusted_device, "\\\\.\\%s", port_name);
    //log( Debug ) << "Create File with device name " << adjusted_device << endlog(); 
    hCom_ = CreateFileA(adjusted_device, GENERIC_READ | GENERIC_WRITE, 0,
                        NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if( hCom_ == INVALID_HANDLE_VALUE )
      HOKUYO_EXCEPT(hokuyo::Exception, "Unable to open serial port. The port you specified (%s) may not be a serial port.", port_name);
    
    //log( Debug ) << "Try SetupComm" << endlog(); 
    SetupComm(hCom_, 4096 * 8, 4096);
    //log( Debug ) << "SetupComm OK" << endlog(); 


    long baudrate_value = CBR_115200;
    DCB dcb;
    GetCommState(hCom_, &dcb);
    dcb.BaudRate = baudrate_value;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.fParity = FALSE;
    dcb.StopBits = ONESTOPBIT;    
    //log( Debug ) << "Set Comm State (baudrate=" << baudrate_value << ")" << endlog(); 
    if (SetCommState(hCom_, &dcb) == 0) {
      PurgeComm(hCom_, PURGE_RXABORT | PURGE_TXABORT | PURGE_RXCLEAR | PURGE_TXCLEAR);
      HOKUYO_EXCEPT(hokuyo::Exception, "Unable to set Baudrate to 115200 on %s", port_name);
    }
    //log( Debug ) << "Set Comm State OK" << endlog(); 

    //log( Debug ) << "" << endlog();
    //log( Debug ) << "###################################################################" << endlog(); 
    //log( Debug ) << "Device is open. Begin configuration..." << endlog(); 
    //log( Debug ) << "###################################################################" << endlog(); 


    //#################################################


    xde::sys::Timer::Sleep(500);

    // Some models (04LX) need to be told to go into SCIP2 mode...
    //log( Debug ) << "Try flushing" << endlog(); 
    laserFlush();
    //log( Debug ) << "Flushing OK" << endlog(); 
    // Just in case a previous failure mode has left our Hokuyo
    // spewing data, we send reset the laser to be safe.
    try {
      //log( Debug ) << "Try resetting" << endlog(); 
      reset();
      //log( Debug ) << "Reset OK" << endlog(); 
    }
    catch (hokuyo::Exception &)
    { 
      // This might be a device that needs to be explicitely placed in
      // SCIP2 mode. 
      // Note: Not tested: a device that is currently scanning in SCIP1.1
      // mode might not manage to switch to SCIP2.0.
      
      //log( Debug ) << "Exception occured : This might be a device that needs to be explicitely placed in SCIP2 mode before reseting. So we do." << endlog(); 
      //log( Debug ) << "Try set to SCIP2" << endlog(); 
      setToSCIP2(); // If this fails then it wasn't a device that could be switched to SCIP2.
      //log( Debug ) << "Set to SCIP2 OK" << endlog(); 
      //log( Debug ) << "Try resetting" << endlog(); 
      reset(); // If this one fails, it really is an error.
      //log( Debug ) << "Reset OK" << endlog(); 
    }
    
    xde::sys::Timer::Sleep(500);
    
    //log( Debug ) << "Try query sensor config" << endlog(); 
    querySensorConfig();
    //log( Debug ) << "Query sensor config OK" << endlog(); 
    
    //log( Debug ) << "Try query version information" << endlog(); 
    queryVersionInformation(); // In preparation for calls to get various parts of the version info.
    //log( Debug ) << "Query version information OK" << endlog(); 
    
    //log( Debug ) << "" << endlog();
    //log( Debug ) << "###################################################################" << endlog(); 
    //log( Debug ) << "Device is configured" << endlog(); 
    //log( Debug ) << "###################################################################" << endlog(); 
  }
  catch (hokuyo::Exception& e)
  {
    //log( Debug ) << "Catch exception : " << e.what() << endlog(); 
    // These exceptions mean something failed on open and we should close
    if (portOpen())
    {
      //log( Debug ) << "Port is open : try CloseHandle" << endlog(); 
      CloseHandle(hCom_);
      //log( Debug ) << "CloseHandle OK" << endlog(); 
    }
    hCom_ = INVALID_HANDLE_VALUE;
    throw e;
  }
}


void hokuyo::Laser::reset ()
{
  //Logger::In inLogger("hokuyo::Laser::reset");

	if (!portOpen())
	    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");
  
  //log( Debug ) << "Try laserFlush" << endlog(); 
  laserFlush();
  //log( Debug ) << "laserFlush OK" << endlog(); 
  try
  {
    //log( Debug ) << "sendCmd TM2" << endlog(); 
    sendCmd("TM2", 3);
    //log( Debug ) << "sendCmd TM2 OK" << endlog(); 
  }
  catch (hokuyo::Exception &) 
  {
  } // Ignore. If the laser was scanning TM2 would fail
  try
  {
    //log( Debug ) << "sendCmd RS" << endlog(); 
    sendCmd("RS", 3);
    last_time_ = 0; // RS resets the hokuyo clock.
    wrapped_ = 0; // RS resets the hokuyo clock.
  }
  catch (hokuyo::Exception &)
  {} // Ignore. If the command coincided with a scan we might get garbage.
  
  //log( Debug ) << "Try laserFlush" << endlog(); 
  laserFlush();
  //log( Debug ) << "laserFlush OK" << endlog(); 
  
  //log( Debug ) << "Try sendCmd RS" << endlog(); 
  sendCmd("RS", 3); // This one should just work.
  //log( Debug ) << "sendCmd OK" << endlog(); 
}


void hokuyo::Laser::close ()
{
  //Logger::In inLogger("hokuyo::Laser::close");
  int retval = 0;

  if (portOpen()) {
    //Try to be a good citizen and completely shut down the laser before we shutdown communication
    try
    {
      //log( Debug ) << "Try reset" << endlog(); 
      reset();
      //log( Debug ) << "Reset Ok" << endlog(); 
    }
    catch (hokuyo::Exception& ) {
      //Exceptions here can be safely ignored since we are closing the port anyways
    }

    //retval = ::close(laser_fd_); // Automatically releases the lock.
    
    //log( Debug ) << "Try CloseHandle" << endlog(); 
    retval = CloseHandle(hCom_);
    //log( Debug ) << "CloseHandle Ok" << endlog(); 
  }

  //laser_fd_ = -1;  
  hCom_ = INVALID_HANDLE_VALUE;

  if (retval == false)
    HOKUYO_EXCEPT(hokuyo::Exception, "Failed to close port properly ");
    
}

void hokuyo::Laser::setToSCIP2()
{
  //Logger::In inLogger("hokuyo::Laser::setToSCIP2");

	if (!portOpen())
	    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");
	const char * cmd = "SCIP2.0";
	char buf[100];
  
  //log( Debug ) << "Try laserWrite cmd=" << cmd  << endlog(); 
	laserWrite(cmd);
  laserWrite("\n");
  //log( Debug ) << "laserWrite OK" << endlog(); 
  
  //log( Debug ) << "Try laserReadline" << endlog(); 
	laserReadline(buf, 100, 10);
  //log( Debug ) << "laserReadline OK - buf=" << buf << endlog(); 
}


int hokuyo::Laser::sendCmd(const char* cmd, int nbMaxAttempts)
{
  //Logger::In inLogger("hokuyo::Laser::sendCmd");

  //log( Debug ) << "Entering sendCmd with cmd=" << cmd << " and nbMaxAttempts=" << nbMaxAttempts << endlog(); 

  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  char buf[100]; 
  
  //log( Debug ) << "Try laserWrite with cmd " << cmd << endlog(); 
  laserWrite(cmd);
  laserWrite("\n");
  //log( Debug ) << "laserWrite OK" << endlog(); 
  
  //log( Debug ) << "Try laserReadlineAfter len=100 cmd=" << cmd << " nbMaxAttempts=" << nbMaxAttempts << endlog(); 
  laserReadlineAfter(buf, 100, cmd, nbMaxAttempts);
  //log( Debug ) << "laserReadlineAfter OK : buf=" << buf << endlog(); 
  
  //log( Debug ) << "Try laserReadline len=100 nbMaxAttempts=" << nbMaxAttempts << endlog(); 
  laserReadline(buf,100,nbMaxAttempts);
  //log( Debug ) << "laserReadline OK : buf=" << buf << endlog(); 

  if (!checkSum(buf,4))
    HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on status code.");
  //log( Debug ) << "checkSum OK" << endlog(); 

  buf[2] = 0;
  
  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    HOKUYO_EXCEPT(hokuyo::Exception, "Hokuyo error code returned. Cmd: %s --  Error: %s", cmd, buf);
}


void hokuyo::Laser::getConfig(LaserConfig& config)
{
  config.min_angle  =  (amin_ - afrt_) * (2.0*M_PI)/(ares_);
  config.max_angle  =  (amax_ - afrt_) * (2.0*M_PI)/(ares_);
  config.ang_increment =  (2.0*M_PI)/(ares_);
  config.time_increment = (60.0)/(double)(rate_ * ares_);
  config.scan_time = 60.0/((double)(rate_));
  config.min_range  =  dmin_ / 1000.0;
  config.max_range  =  dmax_ / 1000.0;
}


int hokuyo::Laser::laserWrite(const char* msg)
{
  /*
  // IO is currently non-blocking. This is what we want for the more common read case.
  int origflags = fcntl(laser_fd_,F_GETFL,0);
  fcntl(laser_fd_, F_SETFL, origflags & ~O_NONBLOCK); // @todo can we make this all work in non-blocking?
  ssize_t len = strlen(msg);
  ssize_t retval = write(laser_fd_, msg, len);
  int fputserrno = errno;
  fcntl(laser_fd_, F_SETFL, origflags | O_NONBLOCK);
  errno = fputserrno; // Don't want to see the fcntl errno below.
  */
  // <= RawSerialDevice::send()

  //Logger::In inLogger("hokuyo::Laser::laserWrite");

  DWORD n;
  //log( Debug ) << "Try WriteFile with msg : " << msg << endlog(); 
  bool retval = WriteFile(hCom_, msg, (DWORD)strlen(msg), &n, NULL);
  
  if (retval == true)
  {
    //log( Debug ) << "WriteFile OK. " << n << " bytes have been writen." << endlog(); 
    return strlen(msg);
  }
  else
    HOKUYO_EXCEPT(hokuyo::Exception, "laserWrite failed");

  return 0;
}


int hokuyo::Laser::laserFlush()
{
  /*
  int retval = tcflush(laser_fd_, TCIOFLUSH);
  */

  // <= fin de RawSerialDevice::flush()
  //************************************
  
  //Logger::In inLogger("hokuyo::Laser::laserFlush");
  
  //log( Debug ) << "Try PurgeComm" << endlog(); 
  bool retval = PurgeComm(hCom_,PURGE_RXABORT | PURGE_TXABORT |
                  PURGE_RXCLEAR | PURGE_TXCLEAR);

  if (retval == false)
    HOKUYO_EXCEPT(hokuyo::Exception, "laserFlush failed");

  //log( Debug ) << "PurgeComm OK" << endlog(); 
  read_buf_start = 0;
  read_buf_end = 0;
  
  return 1;
} 


int hokuyo::Laser::laserReadline(char *buf, int len, int nbMaxAttempts)
{
  //Logger::In inLogger("hokuyo::Laser::laserReadline");
  int current=0;

  /*struct pollfd ufd[1];
  int retval;
  ufd[0].fd = laser_fd_;
  ufd[0].events = POLLIN;
    
  if (timeout == 0)
    timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
    */

  int nbAttempts = 0;
  while (true)
  {
    if (read_buf_start == read_buf_end) // Need to read?
    {
      /*if ((retval = poll(ufd, 1, timeout)) < 0)
        HOKUYO_EXCEPT(hokuyo::Exception, "poll failed   --  error = %d: %s", errno, strerror(errno));*/
      
      /*log( Debug ) << "Try WaitCommEvent : "  << endlog(); 
      LPDWORD lpEvtMask;
      if( false == WaitCommEvent(hCom_, lpEvtMask, NULL) )
        HOKUYO_EXCEPT(hokuyo::Exception, "WaitCommEvent failed with error %d", GetLastError());
      log( Debug ) << "Event detected : 0x" << std::hex << lpEvtMask << endlog(); */

      /*if (retval == 0)
        HOKUYO_EXCEPT(hokuyo::TimeoutException, "timeout reached");
      
      if (ufd[0].revents & POLLERR)
        HOKUYO_EXCEPT(hokuyo::Exception, "error on socket, possibly unplugged");*/
      
      /*int bytes = read(laser_fd_, read_buf, sizeof(read_buf));
      if (bytes == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
        HOKUYO_EXCEPT(hokuyo::Exception, "read failed");*/

      DWORD n;
      //log( Debug ) << "Try ReadFile" << endlog();
      if( false == ReadFile(hCom_, read_buf, sizeof(read_buf), &n, NULL))
      {
        HOKUYO_EXCEPT(hokuyo::Exception, "read failed");
      }
      //log( Debug ) << "ReadFile OK" << endlog();

      nbAttempts++;
      if( nbMaxAttempts > 0 &&  nbAttempts >= nbMaxAttempts )
        HOKUYO_EXCEPT(hokuyo::TimeoutException, "Number max of attempts reached while reading data");
      
      read_buf_start = 0;
      read_buf_end = n;
    }
       
    while (read_buf_end != read_buf_start)
    {
      if (current == len - 1)
      {
        buf[current] = 0;
        HOKUYO_EXCEPT(hokuyo::Exception, "buffer filled without end of line being found");
      }

      buf[current] = read_buf[read_buf_start++];
      if (buf[current++] == '\n')
      {
        buf[current] = 0;
        return current;
      }
    }

  }

  return 0;
}


char* hokuyo::Laser::laserReadlineAfter(char* buf, int len, const char* str, int nbMaxAttempts)
{
  //Logger::In inLogger("hokuyo::Laser::laserReadlineAfter");
  buf[0] = 0;
  char* ind = &buf[0];

  int bytes_read = 0;
  int skipped = 0;

  while ((strncmp(buf, str, strlen(str))) != 0) {
    //log( Debug ) << "Try laserReadline" << endlog();
    bytes_read = laserReadline(buf,len,nbMaxAttempts);
    //log( Debug ) << "bytes_read :" << bytes_read << " buf: " << buf << endlog();

    if ((skipped += bytes_read) > MAX_SKIPPED)
      HOKUYO_EXCEPT(hokuyo::Exception, "too many bytes skipped while searching for match");
  }

  return ind += strlen(str);
}


void hokuyo::Laser::querySensorConfig()
{
  //Logger::In inLogger("hokuyo::Laser::querySensorConfig");

  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  //log( Debug ) << "Try sendCmd PP" << endlog();
  if (sendCmd("PP",10) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting configuration information");
  //log( Debug ) << "sendCmd OK" << endlog();

  char buf[100];
  char* ind;
    
  //log( Debug ) << "Try read DMIN" << endlog();
  ind = laserReadlineAfter(buf,100,"DMIN:",-1);
  sscanf(ind, "%d", &dmin_);
  //log( Debug ) << "Read OK. DMIN=" << dmin_ << endlog();
    
  //log( Debug ) << "Try read DMAX" << endlog();
  ind = laserReadlineAfter(buf,100,"DMAX:",-1);
  sscanf(ind, "%d", &dmax_);
  //log( Debug ) << "Read OK. DMAX=" << dmax_ << endlog();
    
  //log( Debug ) << "Try read ARES" << endlog();
  ind = laserReadlineAfter(buf,100,"ARES:",-1);
  sscanf(ind, "%d", &ares_);
  //log( Debug ) << "Read OK. ARES=" << ares_ << endlog();
    
  //log( Debug ) << "Try read AMIN" << endlog();
  ind = laserReadlineAfter(buf,100,"AMIN:",-1);
  sscanf(ind, "%d", &amin_);
  //log( Debug ) << "Read OK. AMIN=" << amin_ << endlog();
    
  //log( Debug ) << "Try read AMAX" << endlog();
  ind = laserReadlineAfter(buf,100,"AMAX:",-1);
  sscanf(ind, "%d", &amax_);
  //log( Debug ) << "Read OK. AMAX=" << amax_ << endlog();
    
  //log( Debug ) << "Try read AFRT" << endlog();
  ind = laserReadlineAfter(buf,100,"AFRT:",-1);
  sscanf(ind, "%d", &afrt_);
  //log( Debug ) << "Read OK. AFRT=" << afrt_ << endlog();
    
  //log( Debug ) << "Try read SCAN" << endlog();
  ind = laserReadlineAfter(buf,100,"SCAN:",-1);
  sscanf(ind, "%d", &rate_);
  //log( Debug ) << "Read OK. SCAN=" << rate_ << endlog();
    
  return;
}


bool hokuyo::Laser::checkSum(const char* buf, int buf_len)
{
  char sum = 0;
  for (int i = 0; i < buf_len - 2; i++)
    sum += (unsigned char)(buf[i]);

  if ((sum & 63) + 0x30 == buf[buf_len - 2])
    return true;
  else
    return false;
}


uint64_t hokuyo::Laser::readTime(int nbMaxAttempts)
{
  char buf[100];

  laserReadline(buf, 100, nbMaxAttempts);
  if (!checkSum(buf, 6))
    HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on time stamp.");

  unsigned int laser_time = ((buf[0]-0x30) << 18) | ((buf[1]-0x30) << 12) | ((buf[2]-0x30) << 6) | (buf[3] - 0x30);

  if (laser_time == last_time_)
  {
    if (++time_repeat_count_ > 2)
    {
      HOKUYO_EXCEPT(hokuyo::RepeatedTimeException, "The timestamp has not changed for %d reads", time_repeat_count_);
    }
  }
  else
  {
    time_repeat_count_ = 0;
  }

  if (laser_time < last_time_)
    wrapped_++;
  
  last_time_ = laser_time;
  
  return (uint64_t)((wrapped_ << 24) | laser_time)*(uint64_t)(1000000);
}


void hokuyo::Laser::readData(hokuyo::LaserScan& scan, bool has_intensity, int nbMaxAttempts)
{
  scan.ranges.clear();
  scan.intensities.clear();

  int data_size = 3;
  if (has_intensity)
    data_size = 6;

  char buf[100];

  int ind = 0;

  scan.self_time_stamp = readTime(nbMaxAttempts);

  int bytes;

  float range;
  float intensity;

  for (;;)
  {
    bytes = laserReadline(&buf[ind], 100 - ind, nbMaxAttempts);
    
    if (bytes == 1)          // This is \n\n so we should be done
      return;
    
    if (!checkSum(&buf[ind], bytes))
      HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on data read.");
    
    bytes += ind - 2;
    
    // Read as many ranges as we can get
    for (int j = 0; j < bytes - (bytes % data_size); j+=data_size)
    {
      if (scan.ranges.size() < MAX_READINGS)
      {
        range = (((buf[j]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30)) / 1000.0;
	scan.ranges.push_back(range);

        if (has_intensity)
        {
	  intensity = (((buf[j+3]-0x30) << 12) | ((buf[j+4]-0x30) << 6) | (buf[j+5]-0x30));
          scan.intensities.push_back(intensity);
        }
      }
      else
      {
        HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Got more readings than expected");
      }
    }
    // Shuffle remaining bytes to front of buffer to get them on the next loop
    ind = 0;
    for (int j = bytes - (bytes % data_size); j < bytes ; j++)
      buf[ind++] = buf[j];
  }
}


int hokuyo::Laser::pollScan(hokuyo::LaserScan& scan, double min_ang, double max_ang, int cluster, int timeout)
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  int status;

  // Always clear ranges/intensities so we can return easily in case of erro
  scan.ranges.clear();
  scan.intensities.clear();

  // clustering of 0 and 1 are actually the same
  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);
  
  status = sendCmd(cmdbuf, timeout);
  
  scan.system_time_stamp = timeHelper() + offset_;
  
  if (status != 0)
    return status;
  
  // Populate configuration
  scan.config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan.config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan.config.scan_time = 0.0;
  scan.config.min_range  =  dmin_ / 1000.0;
  scan.config.max_range  =  dmax_ / 1000.0;
  
  readData(scan, false, timeout);
  
  long long inc = (long long)(min_i * scan.config.time_increment * 1000000000);
  
  scan.system_time_stamp += inc;
  scan.self_time_stamp += inc;
  return 0;  
}

int hokuyo::Laser::laserOn() {
  int res = sendCmd("BM",10);
  if (res == 1)
    HOKUYO_EXCEPT(hokuyo::Exception, "Unable to control laser due to malfunction.");
  return res;
}

int hokuyo::Laser::laserOff() {
  return sendCmd("QT",10);
}

int hokuyo::Laser::stopScanning() {
  try {
    return laserOff();
  }
  catch (hokuyo::Exception &)
  {
    // Ignore exception because we might have gotten part of a scan 
    // instead of the expected response, which shows up as a bad checksum.
    laserFlush();
  } 
  return laserOff(); // This one should work because the scan is stopped.
}

int hokuyo::Laser::requestScans(bool intensity, double min_ang, double max_ang, int cluster, int skip, int count)
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  //! @todo check that values are within range?

  int status;

  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  char intensity_char = 'D';
  if (intensity)
    intensity_char = 'E';
  
  sprintf(cmdbuf,"M%c%.4d%.4d%.2d%.1d%.2d", intensity_char, min_i, max_i, cluster, skip, count);
  
  status = sendCmd(cmdbuf);
  
  return status;
}

bool hokuyo::Laser::isIntensitySupported()
{
  hokuyo::LaserScan  scan;
  
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  // Try an intensity command.
  try
  {
    requestScans(1, 0, 0, 0, 0, 1);
    serviceScan(scan);
    return true;
  }
  catch (hokuyo::Exception &)
  {} 
  
  // Try a non intensity command.
  try
  {
    requestScans(0, 0, 0, 0, 0, 1);
    serviceScan(scan);
    return false;
  }
  catch (hokuyo::Exception &)
  {
    HOKUYO_EXCEPT(hokuyo::Exception, "Exception whil trying to determine if intensity scans are supported.")
  } 
}

int hokuyo::Laser::serviceScan(hokuyo::LaserScan& scan)
{
  //Logger::In inLogger("hokuyo::Laser::serviceScan");

  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  // Always clear ranges/intensities so we can return easily in case of erro
  scan.ranges.clear();
  scan.intensities.clear();
  //log( Debug ) << "ranges/intensities cleared" << endlog();

  char buf[100];

  bool intensity = false;
  int min_i;
  int max_i;
  int cluster;
  int skip;
  int left;

  char* ind;

  int status = -1;

  do {
    //log( Debug ) << "Try laserReadlineAfter" << endlog();
    ind = laserReadlineAfter(buf, 100, "M");
    scan.system_time_stamp = timeHelper() + offset_;
    //log( Debug ) << "laserReadlineAfter OK" << endlog();

    if (ind[0] == 'D')
      intensity = false;
    else if (ind[0] == 'E')
      intensity = true;
    else
      continue;

    ind++;

    sscanf(ind, "%4d%4d%2d%1d%2d", &min_i, &max_i, &cluster, &skip, &left); 
    
    //log( Debug ) << "Try laserReadline" << endlog();
    laserReadline(buf, 100);
    //log( Debug ) << "laserReadline OK" << endlog();

    buf[4] = 0;
    
    if (!checkSum(buf, 4))
      HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on status code: %s", buf);
    //log( Debug ) << "checkSum OK" << endlog();

    sscanf(buf, "%2d", &status);
    
    //log( Debug ) << "status : " << status << endlog();

    if (status != 99)
      return status;
    
  } while(status != 99);
    
  scan.config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan.config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan.config.scan_time = (60.0 * (skip + 1))/((double)(rate_));
  scan.config.min_range  =  dmin_ / 1000.0;
  scan.config.max_range  =  dmax_ / 1000.0;
  
  //log( Debug ) << "Try readData" << endlog();
  readData(scan, intensity);
  //log( Debug ) << "readData OK" << endlog();

  long long inc = (long long)(min_i * scan.config.time_increment * 1000000000);

  scan.system_time_stamp += inc;
  scan.self_time_stamp += inc;

  return 0;
}

void hokuyo::Laser::queryVersionInformation()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  if (sendCmd("VV",10) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting version information");
  
  char buf[100];
  vendor_name_ = laserReadlineAfter(buf, 100, "VEND:");
  vendor_name_ = vendor_name_.substr(0,vendor_name_.length() - 3);
  
  product_name_ = laserReadlineAfter(buf, 100, "PROD:");
  product_name_ = product_name_.substr(0,product_name_.length() - 3);
  
  firmware_version_ = laserReadlineAfter(buf, 100, "FIRM:");
  firmware_version_ = firmware_version_.substr(0,firmware_version_.length() - 3);

  protocol_version_ = laserReadlineAfter(buf, 100, "PROT:");
  protocol_version_ = protocol_version_.substr(0,protocol_version_.length() - 3);
  
  // This crazy naming scheme is for backward compatibility. Initially
  // the serial number always started with an H. Then it got changed to a
  // zero. For a while the driver was removing the leading zero in the
  // serial number. This is fine as long as it is indeed a zero in front.
  // The current behavior is backward compatible but will accomodate full
  // length serial numbers.
  serial_number_ = laserReadlineAfter(buf, 100, "SERI:");
  serial_number_ = serial_number_.substr(0,serial_number_.length() - 3);
  if (serial_number_[0] == '0')
    serial_number_[0] = 'H';
  else if (serial_number_[0] != 'H')
    serial_number_ = 'H' + serial_number_;
}


std::string hokuyo::Laser::getID()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return serial_number_;
}


std::string hokuyo::Laser::getFirmwareVersion()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return firmware_version_;
}


std::string hokuyo::Laser::getProtocolVersion()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return protocol_version_;
}


std::string hokuyo::Laser::getVendorName()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return vendor_name_;
}


std::string hokuyo::Laser::getProductName()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return product_name_;
}


std::string hokuyo::Laser::getStatus()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  if (sendCmd("II",10) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting device information information");
  
  char buf[100];
  char* stat = laserReadlineAfter(buf, 100, "STAT:");

  std::string statstr(stat);
  statstr = statstr.substr(0,statstr.length() - 3);

  return statstr;
}
    
template <class C> 
C median(std::vector<C> &v)
{
  std::vector<long long int>::iterator start  = v.begin();
  std::vector<long long int>::iterator end    = v.end();
  std::vector<long long int>::iterator median = start + (end - start) / 2;
  std::nth_element(start, median, end);
  return *median;
}


long long int hokuyo::Laser::getHokuyoClockOffset(int reps)
{
  std::vector<long long int> offset(reps);

  sendCmd("TM0", 10);
  for (int i = 0; i < reps; i++)
  {
    long long int prestamp = timeHelper();
    sendCmd("TM1", 10);
    long long int hokuyostamp = readTime();
    long long int poststamp = timeHelper();
    offset[i] = hokuyostamp - (prestamp + poststamp) / 2;
  }
  sendCmd("TM2", 10);
  
  long long out = median(offset);

  return out;
}

long long int hokuyo::Laser::getHokuyoScanStampToSystemStampOffset(bool intensity, double min_ang, double max_ang, int clustering, int skip, int reps)
{
  if (reps < 1)
    reps = 1;
  else if (reps > 99) 
    reps = 99;
  
  std::vector<long long int> offset(reps);
  
  if (requestScans(intensity, min_ang, max_ang, clustering, skip, reps) != 0)
  {
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting scan while caliblating time.");
    return 1;
  }

  hokuyo::LaserScan scan;
  for (int i = 0; i < reps; i++)
  {
    serviceScan(scan);
    offset[i] = scan.self_time_stamp - scan.system_time_stamp;
  }

  return median(offset);
}

long long hokuyo::Laser::calcLatency(bool intensity, double min_ang, double max_ang, int clustering, int skip, int num)
{
  offset_ = 0;
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  if (num <= 0)
    num = 10;
  
  int ckreps = 1;
  int scanreps = 1;
  long long int start = getHokuyoClockOffset(ckreps);
  long long int pre = 0;
  std::vector<long long int> samples(num);
  for (int i = 0; i < num; i++)
  {                                                 
    long long int scan = getHokuyoScanStampToSystemStampOffset(intensity, min_ang, max_ang, clustering, skip, scanreps) - start; 
    long long int post = getHokuyoClockOffset(ckreps) - start;
    samples[i] = scan - (post+pre)/2;
    pre = post;
  }

  offset_ = median(samples);
  return offset_;
}
