/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: LS01B
@filename: ls01b.h
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-21     fu          new
*******************************************************/
#ifndef LS01B_H
#define LS01B_H
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <string>
#include "ls01b_v2/lsiosr.h"
#include <ros/ros.h>

namespace ls {

typedef struct {
    double degree;
    double range;
    double intensity;
} ScanPoint;

//typedef struct {
//  std::chrono::steady_clock::time_point time_stamp;
//  std::vector<ScanPoint> points;
//}ScanMsg;

class LS01B
{
public:
  ~LS01B();
    /**
    * 实例化雷达
    * port: 串口号，
    * baud_rate: 波特率 460800
    * resolution: 雷达角度分辨率1.0度 0.5度 0.25度
    */
    static LS01B* instance(std::string port, int baud_rate, double resolution);

    /**
    * 判断雷达是否有效
    */
    bool isHealth();

    /**
     * 恢复雷达状态
     */
    bool resetHealth();

    /**
    * 获取雷达数据
    * poins: 雷达点的数据。类型为ScanPoint数组
    */
    int getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration);

    /**
    * 获取软件版本号
    * version: 返回版本号
    */
    int getVersion(std::string &version);


    int getRate();

    /**
     * start lidar, System work normally
     */
    int startScan();

    /**
     * stop lidar, System in sleep mode
     */
    int stopScan();

    /**
     * set scan mode. continouse or once
     * is_continuous:
     */
    int setScanMode(bool is_continuous);

    /**
     * stop recv data from lidar
     */
    int stopRecvData();

    /**
     * switch angle or intensity in recv data
     * use_angle: true. the angle is chosed
     *            false. the intensity is chosed
     */
    int switchData(bool use_angle);

    /**
     * set the motor spped
     * rpm: round per minute
     */
    int setMotorSpeed(int rpm);

    /**
    * 设置雷达数据分辨率
    * resolution: 分辨率
    */
    int setResolution(double resolution);

    double getRPM();

private:
    LS01B(std::string port, int baud_rate, double resolution = 0.25);

    void recvThread();

    uint16_t checkSum(const uint8_t *p_byte);

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;
//    ScanMsg scan_msg_;

    LSIOSR * serial_;
    boost::thread *recv_thread_;
    boost::mutex mutex_;

    bool is_shutdown_;    // shutdown recvthread
    bool is_start_;       // begin to read data from lidar
    bool use_angle_;
    
    int scan_health_; // 0 OK
    double resolution_;
    int data_len_;

    int points_size_;
    int rpm_;
    double real_rpm_;

    ros::Time pre_time_;
    ros::Time time_;

};

}
#endif
