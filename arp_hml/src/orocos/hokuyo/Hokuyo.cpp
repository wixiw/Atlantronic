#include "Hokuyo.h"

#include <rtt/Logger.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <string>

#include "sys/timer.h"

using namespace std;
using namespace RTT;
using namespace Eigen;

namespace dio {

  Hokuyo::Hokuyo(const std::string & name, const std::string & comPort) : 
Hardware(name),
out_port("scan"),
min_angle(-M_PI/2.),
max_angle(M_PI/2.),
intensity(false),
cluster(1),
skip(0)
{
  Logger::In inLogger(this->getName());
  
  this->setActivity(new RTT::Activity(0, 0.000001));
  this->getActivity<RTT::Activity>()->thread()->setWaitPeriodPolicy(ORO_WAIT_REL);
  this->getActivity<RTT::Activity>()->thread()->setPriority(RTT::os::IncreasePriority);

  //         )
  //      (
  //        ,
  //     ___)\
  //    (_____)
  //   (_______) 
  //
  this->addOperation("stop", &Hokuyo::doStop, this, RTT::OwnThread);
  
  // add ports
  this->ports()->addPort( out_port );

  // Connexion to Hokuyo
  try
  {
    deviceStatus =  "unknown";

    lrf.open( comPort.c_str() );
    log( Info ) << "Device opened successfully." << endlog();

    deviceId = lrf.getID();
    vendorName = lrf.getVendorName();
    firmwareVersion = lrf.getFirmwareVersion();
    productName = lrf.getProductName();
    protocolVersion = lrf.getProtocolVersion();

    log( Info ) << "Device ID : " << deviceId << endlog();
    log( Info ) << "Vendor Name : " << vendorName << endlog();
    log( Info ) << "Firmware Version : " << firmwareVersion << endlog();
    log( Info ) << "Product Name : " << productName << endlog();
    log( Info ) << "Protocol Version : " << protocolVersion << endlog();

    deviceStatus = lrf.getStatus();
    if(deviceStatus != std::string("Sensor works well."))
    {
      log( Critical ) << "Laser returned abnormal status message (" << deviceStatus << ") => aborting" << endlog();
      this->Fatal();
    }
    log( Info ) << "Device Status : " << deviceStatus << endlog();
  }
  catch(std::runtime_error &e)
  {
    log( Critical )  << "Exception thrown while opening Hokuyo : " << e.what() << endlog();
    this->Fatal();
  }
}

Hokuyo::~Hokuyo()
{
  Logger::In inLogger(this->getName());

  this->ports()->clear();

  try
  {
    lrf.close();
    log( Info ) << "Device closed successfully." << endlog();
  }
  catch(std::runtime_error &e)
  {
    log( Critical )  << "Exception thrown while trying to close Hokuyo : " << e.what() << endlog();
  }
}

void Hokuyo::doStop()
{
  this->Stop();
}

void Hokuyo::SetParameters(const boost::property_tree::ptree & param) 
{
  Logger::In inLogger(this->getName());
  
  {
    double min_angle_ = min_angle;
    double max_angle_ = max_angle;

    try{
    min_angle_ = param.get<double>("min_angle");
    } catch(...){}
  
    try{
    max_angle_ = param.get<double>("max_angle");
    } catch(...){}

    if(max_angle_ < min_angle_)
      log( Warning ) << "max_angle (" << max_angle_ << ") should be greater than min_angle (" << min_angle_ << ")" << endlog();
    else
    {
      min_angle = min_angle_;
      max_angle = max_angle_;
      log( Info ) << "min_angle changed to " << min_angle << endlog();
      log( Info ) << "max_angle changed to " << max_angle << endlog();
    }
  }

  try{
  intensity = param.get<bool>("intensity");
  log( Info ) << "intensity changed to " << intensity << endlog();
  } catch(...){}
  
  try{
  cluster = param.get<int>("cluster");
  log( Info ) << "cluster changed to " << cluster << endlog();
  } catch(...){}
  
  try{
  skip = param.get<int>("skip");
  log( Info ) << "skip changed to " << skip << endlog();
  } catch(...){}
}

bool Hokuyo::ConfigureHook() 
{
  Logger::In inLogger(this->getName());
  
  try
  {
    log( Debug ) << "Try turning laser ON" << endlog();
    lrf.laserOn();
    log( Info ) << "Laser is turned ON" << endlog();
    
    log( Debug ) << "Try requestScans" << endlog();
    int status = lrf.requestScans(intensity, min_angle, max_angle, cluster, skip);
    log( Debug ) << "requestScans OK" << endlog();

    if (status != 0) {
      log( Error ) << "Failed to request scans from device (Status: " << status << ")" << endlog();
      return false;
    }

    log( Info ) << "Waiting for first scan (sample data)..." << endlog();
    hokuyo::LaserScan scan;
    
    while(1)
    {
      try
      {
        log( Debug ) << "Try serviceScan" << endlog();
        status = lrf.serviceScan(scan);
        log( Debug ) << "serviceScan OK" << endlog();
        if(status != 0)
        {
          log( Warning ) << "Error getting scan (Status: " << status << ")" << endlog();
          break;
        }
      } 
      catch (hokuyo::CorruptedDataException &e) 
      {
        log( Warning ) << "Skipping corrupted data" << endlog();
        continue;
      } 
      catch (hokuyo::Exception& e) 
      {
        log( Error ) << "Exception thrown while trying to get scan : " << e.what() << endlog();
        return false;
      }
      break;
    }

    if( status == 0 )
    {
      log( Info ) << "Nb data : " << scan.ranges.size() << endlog();
      log( Info ) << "Nb angle increments : " << 1 + (scan.config.max_angle - scan.config.min_angle) / scan.config.ang_increment << endlog();
      log( Info ) << "min_angle : " << scan.config.min_angle << endlog();
      log( Info ) << "max_angle : " << scan.config.max_angle << endlog();
      log( Info ) << "ang_increment : " << scan.config.ang_increment << endlog();
      log( Info ) << "min_range : " << scan.config.min_range << endlog();
      log( Info ) << "max_range : " << scan.config.max_range << endlog();
      log( Info ) << "time_increment : " << scan.config.time_increment << endlog();
      log( Info ) << "scan_time : " << scan.config.scan_time << endlog();
      log( Info ) << "system_time_stamp : " << scan.system_time_stamp << endlog();
      log( Info ) << "self_time_stamp : " << scan.self_time_stamp << endlog();

      mat = Eigen::MatrixXd::Random(2,scan.ranges.size());
      out_port.setDataSample( mat );
    }

    try
    {
      lrf.stopScanning(); // This actually just calls laser Off internally.
      log( Info ) << "Laser is turned OFF" << endlog();
    } 
    catch (hokuyo::Exception &e)
    {
      log( Error ) << "Exception thrown while trying to stop scan. " << e.what() << endlog();
      return false;
    }
    return (status == 0);
  }
  catch(std::runtime_error &e)
  {
    log( Error )  << "Exception thrown while configuring Hokuyo : " << e.what() << endlog();
    return false;
  }
  return true;
}

// TODO: Implement Error/Cleanup handling
bool Hokuyo::StartHook() 
{
  Logger::In inLogger(this->getName());

  try
  {
    log( Debug ) << "Try turning laser ON" << endlog();
    lrf.laserOn();
    log( Info ) << "Laser is turned ON" << endlog();
    
    log( Debug ) << "Try requestScans" << endlog();
    int status = lrf.requestScans(intensity, min_angle, max_angle, cluster, skip);
    log( Debug ) << "requestScans OK" << endlog();

    if (status != 0) {
      log( Error ) << "Failed to request scans from device (Status: " << status << ")" << endlog();
      return false;
    }
  }
  catch(std::runtime_error &e)
  {
    log( Error )  << "Exception thrown while configuring Hokuyo : " << e.what() << endlog();
    return false;
  }
  return true;
}

void Hokuyo::StopHook()
{
  Logger::In inLogger(this->getName());

  try
  {
    lrf.stopScanning(); // This actually just calls laser Off internally.
    log( Info ) << "Laser is turned OFF" << endlog();
  }
  catch(std::runtime_error &e)
  {
    log( Error ) << "Exception thrown while trying to stop scan. " << e.what() << endlog();
    return;
  }
  return;
}

void Hokuyo::CleanupHook() { }
void Hokuyo::CloseHook() { }
void Hokuyo::FatalHook() { }

void Hokuyo::UpdateCalibrate()
{
  Logger::In inLogger(this->getName());
}

void Hokuyo::UpdateRun()
{

  hokuyo::LaserScan scan;

  while(1)
  {
    try
    {
      int status = lrf.serviceScan(scan);
      if(status != 0)
      {
		Logger::In inLogger(this->getName());
        log( Warning ) << "Error getting scan (Status: " << status << ")" << endlog();
        return;
      }
    } 
    catch (hokuyo::CorruptedDataException &e) 
    {
	  Logger::In inLogger(this->getName());
      log( Warning ) << "Skipping corrupted data" << endlog();
      continue;
    } 
    catch (hokuyo::Exception& e) 
    {
	  Logger::In inLogger(this->getName());
      log( Error ) << "Exception thrown while trying to get scan : " << e.what() << endlog();
      return;
    }
    break;
  }
  
  for( unsigned int i=0 ; i < scan.ranges.size() ; i++)
  {
    mat(0,i) = scan.config.min_angle + i * scan.config.ang_increment ;
    mat(1,i) = (double) scan.ranges[i];
  }

}

void Hokuyo::WriteOutputPorts()
{
  out_port.write( mat ); 
}

} //namespace