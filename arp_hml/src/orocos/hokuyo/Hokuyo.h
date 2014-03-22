#pragma once

#include "hardware.h"

#include "Eigen/lgsm"

#include <string>
#include <rtt/extras/PeriodicActivity.hpp>
#include <boost/shared_ptr.hpp>

//#include "UrgCtrl.h"
#include "libhokuyo/libhokuyo.h"

namespace dio {

class Hokuyo : public Hardware
{
public:
  // public specific structure

protected:
  // Ports
  RTT::OutputPort<Eigen::MatrixXd>  out_port;

  // Attributes
  double min_angle;
  double max_angle;
  bool   intensity;
  int    cluster;
  int    skip;

public:
  Hokuyo(const std::string & name, const std::string & comPort);
  ~Hokuyo();

  // "Periodic" states
  virtual void UpdateRun();
  virtual void UpdateCalibrate();

  // Hooks on transitions
  virtual bool ConfigureHook();
  virtual bool StartHook();
  virtual void StopHook();
  virtual void CleanupHook();
  virtual void CloseHook();
  virtual void FatalHook();

  virtual void WriteOutputPorts();

  virtual void SetParameters(const boost::property_tree::ptree & param);
  
protected:
  void doStop();

protected:
  hokuyo::Laser lrf;

  std::string deviceId;
  std::string vendorName;
  std::string firmwareVersion;
  std::string productName;
  std::string protocolVersion;
  std::string deviceStatus;

  Eigen::MatrixXd mat;
};

} //namespace