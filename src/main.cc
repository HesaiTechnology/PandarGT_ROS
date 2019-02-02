#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "hesai_gt/hesai_gt.h"

using namespace std;

class HesaiGTClient
{
public:
  HesaiGTClient(ros::NodeHandle node, ros::NodeHandle nh) {
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

    string serverIp;
    int serverPort;
    string calibrationFile;
    int lidarRecvPort;
    string lidarCorrectionFile;
    int channelCount;
    string pcapFile;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("server_port", serverPort);
    nh.getParam("calibration_file", calibrationFile);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("channel_count", channelCount);

    if(!pcapFile.empty())
    {
      hsdk = new HesaiGT(pcapFile, \
          boost::bind(&HesaiGTClient::lidarCallback, this, _1, _2),
          0, std::string("hesaiGT"));
    }
    else if(serverIp.empty())
    {
      hsdk = new HesaiGT("192.168.1.201", lidarRecvPort, \
          boost::bind(&HesaiGTClient::lidarCallback, this, _1, _2),
          0, std::string("hesaiGT"));
    }

    hsdk->LoadCorrectionFile(lidarCorrectionFile);
    hsdk->Start();
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
    pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
    cld->header.frame_id = "hesaiGT";
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
  }

private:
  ros::Publisher lidarPublisher;
  HesaiGT        *hsdk;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hesai_GT_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiGTClient gtClient(node, nh);

  ros::spin();
  return 0;
}

