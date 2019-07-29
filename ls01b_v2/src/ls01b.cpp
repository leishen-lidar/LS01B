/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: LS01B
@filename: ls01b.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-21     fu          new
*******************************************************/
#include "ls01b_v2/ls01b.h"
#include <stdio.h>

namespace ls{
LS01B * LS01B::instance(std::string port, int baud_rate, double resolution)
{
  static LS01B obj(port, baud_rate, resolution);
  return &obj;
}

LS01B::LS01B(std::string port, int baud_rate, double resolution)
{
  scan_health_ = 0;
  resolution_ = resolution;
  is_shutdown_ = false;
  is_start_ = false;
  use_angle_ = true;

  data_len_ = 180;
  points_size_ = 360/resolution;
  scan_points_.resize(points_size_);

  serial_ = LSIOSR::instance(port, baud_rate);
  serial_->init();
  recv_thread_ = new boost::thread(boost::bind(&LS01B::recvThread, this));
}

LS01B::~LS01B()
{
  printf("start ls01b\n");
  is_shutdown_ = true;
  recv_thread_->interrupt();
  recv_thread_->join();

  recv_thread_ = NULL;
  delete recv_thread_;

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end ls01b\n");
}

bool LS01B::isHealth()
{
  return (scan_health_ < 0) ? false : true;
}

bool LS01B::resetHealth()
{
  scan_health_ = 0;
}

int LS01B::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
  // boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;

  scan_duration = (time_ - pre_time_).toSec();
  // ROS_INFO("scan_duration = %f", scan_duration);
}

int LS01B::getVersion(std::string &version)
{
  version = "ls01b_v2_0";
  return 0;
}

double LS01B::getRPM()
{
  return real_rpm_;
}

int LS01B::startScan()
{
  printf("startScan\n");
  char data[3];
  data[0] = 0xa5;
  data[1] = 0x2c;
  data[2] = 0x0;
  int rtn = serial_->send((const char*)data, 2);
  if (rtn < 0)
  {
    printf("start scan error !\n");
    scan_health_ = -1;
    return rtn;
  }
  sleep(1.0);

  return rtn;
}

int LS01B::stopScan()
{
  printf("stopScan\n");
  char data[2];
  data[0] = 0xa5;
  data[1] = 0x25;
  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("stop scan error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  sleep(1.0);
  return rtn;
}

int LS01B::setScanMode(bool is_continuous)
{
  printf("setScanMode is_continuous = %d\n", is_continuous);
  char data[2];
  data[0] = 0xa5;
  data[1] = 0x0;
  
  if (is_continuous){
    data[1] = 0x20;
  }
  else{
    data[1] = 0x22;
  }

  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("setScanMode error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = true;
  sleep(1.0);
  return rtn;
}

int LS01B::stopRecvData()
{
  printf("stopRecvData\n");
  char data[2];
  data[0] = 0xa5;
  data[1] = 0x21;

  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("stopRecvData error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  sleep(1.0);
  return rtn;
}

int LS01B::switchData(bool use_angle)
{
  printf("switchData use_angle = %d\n", use_angle);
  char data[4];
  data[0] =  0xa5;
  data[1] = 0x0;
  data[2] = 0x0;
  data[3] = 0x0;

  if (use_angle){
    data[1] = 0x5c;
  }
  else{
    data[1] = 0x50;
  }

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("switchData error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  use_angle_ = use_angle;
  sleep(1.0);
  return rtn;
}

int LS01B::setMotorSpeed(int rpm)
{
  printf("setMotorSpeed rpm = %d\n", rpm);

  char data[4];
  data[0] = 0xa5;
  data[1] = 0x26;
  data[3] = (rpm & 0xff);
  data[2] = (rpm >> 8) & 0xff;

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("setMotorSpeed error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  sleep(1.0);
  return rtn;
}

int LS01B::setResolution(double resolution)
{
  printf("setResolution resolution = %f\n", resolution);
  char data[4];
  data[0] = 0xa5;
  data[1] = 0x30;
  data[2] = 0x00;

  if (resolution == 0.25){
    data[3] = 0x19;
    data_len_ = 180;
  }
  else if (resolution == 0.5){
    data[3] = 0x32;
    data_len_ = 90;
  }
  else if (resolution == 1.0){
    data[3] = 0x64;
    data_len_ = 45;
  }
  else
  {
    // resolution error
    scan_health_ = -2;
    return scan_health_;
  }

  resolution_ = resolution;
  points_size_ = 360/resolution;
  scan_points_.resize(points_size_);
  

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("setResolution error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  sleep(1.0);

  return rtn;
}

void LS01B::recvThread()
{

  uint8_t start_count = 0;
  char header[6];
  uint8_t temp_char;
  char * packet_bytes = new char[data_len_];
  if (packet_bytes == NULL)
  {
    packet_bytes = NULL;
    // printf("new char [data_len_] error \n");
  }

  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();
  while(!is_shutdown_){
    while(!is_start_){
      usleep(300000);
    }

    int count = serial_->read(&header[start_count], 1);
    if (count<=0)
    {
      // printf("read header[%d] error\n", start_count);
      start_count = 0;
      scan_health_ = -3;
      continue;
    }

    if(0 == start_count){
      if (0xA5 == (header[start_count]&0xff)){
        start_count = 1;
      }
      else{
        start_count = 0;
      }
    }
    else if (1 == start_count) {
      if (0x6A == (header[start_count]&0xff) || 0x5A == (header[start_count]&0xff))
      {
        if (0x6A == (header[start_count]&0xff))
        {
          t2 = boost::posix_time::microsec_clock::universal_time();
          boost::posix_time::millisec_posix_time_system_config::time_duration_type t_elapse;
          t_elapse = t2 - t1;
          real_rpm_ = 1000000.0 / t_elapse.ticks();

          t1 = t2;

          boost::unique_lock<boost::mutex> lock(mutex_);
          scan_points_bak_.resize(scan_points_.size());
          scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
          pre_time_ = time_;
          lock.unlock();

          time_ = ros::Time::now();
        }

        start_count = 2;
        int count = serial_->read(&header[start_count], 4);
        if (count != 4)
        {
          // printf("read header error\n");
          start_count = 0;
          scan_health_ = -3;
          continue;
        }

        rpm_ = ((header[2]&0x7f) << 8) + (header[3]&0xff);
        int flag = ((header[2] & 0x80) == 0) ? 0 : 1;
        int angular_resolution = (header[4]&0xff) >> 1;
        int start_angle = ((header[4] & 0x01) << 8) + (header[5]&0xff);



        count = serial_->read(packet_bytes, data_len_);
        if (count != data_len_)
        {
          // printf("read %d packet error\n", data_len_);
          start_count = 0;
          scan_health_ = -3;
          continue;
        }

        for (int i = 0; i < data_len_; i = i+3)
        {
          // printf("%02X %02X %02X \n", (packet_bytes[i]) & 0xFF, (packet_bytes[i+1]) & 0xFF, (packet_bytes[i+2]) & 0xFF);
          int idx = start_angle / resolution_ + i/3;
          double distance = ((packet_bytes[i+1] & 0xFF) << 8) | (packet_bytes[i+2] & 0xFF);
          // double degree = start_angle + (packet_bytes[i] & 0xFF )* resolution_;
          double degree = start_angle + idx * resolution_;

          boost::unique_lock<boost::mutex> lock(mutex_);
          scan_points_[idx].degree = degree;
          scan_points_[idx].range = distance/1000.0;
          if(use_angle_)
          {
            scan_points_[idx].intensity = 0;
          }
          else
          {
            scan_points_[idx].intensity = packet_bytes[i] & 0xFF;
            // printf("%f ", scan_points_[idx].intensity);
          }
          lock.unlock();
        }
        // printf("\n");

        start_count = 0;

      }
      else{
        start_count = 0;
      }

    }
  }

  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }
}
}

