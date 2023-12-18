#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <stdio.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 

using namespace cv;

using std::placeholders::_1;

using namespace std::chrono_literals;
 
class LocalMap : public rclcpp::Node
{
  public:

    LocalMap()
    : Node("create_local_map")
    {
       subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&LocalMap::topic_callback, this, _1));

       publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_cost_map", 10);

    }
 
  private:


    // This method executes every 500 milliseconds
    void topic_callback(const sensor_msgs::msg::PointCloud2 msg)
    {
      
      int width = 500;
      int height = 500;
      Mat map = Mat::zeros(Size(width,height),CV_8UC1);

      Point p1(0, 0); 
      Point p2(width,height); 
   
      cv::rectangle(map, p1, p2, Scalar(0, 0, 0), -1, LINE_8);

      sensor_msgs::msg::PointCloud out_pointcloud;

      sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
      int x;
      int y;

      for(int i = 0; i < int(msg.width); i ++){
        //std::cout << out_pointcloud.points[i].x << " " << out_pointcloud.points[i].y << " " << out_pointcloud.points[i].z << std::endl;
        
        if(std::isnan(out_pointcloud.points[i].z) == false && out_pointcloud.points[i].z > -1.6 && out_pointcloud.points[i].z < 10){

          x = out_pointcloud.points[i].x*20.0 + width/2;
          y = out_pointcloud.points[i].y*20.0 + height/2;
          Point p1(x, y); 
          cv::circle(map, p1, 5, Scalar(100, 100, 100), -1);
        }
      }


        //imshow("Output", map); 
        //waitKey(1); 
        
        float centrX = width/2;
        float centrY = height/2;

        Mat gray_map = map;

        //cv::cvtColor(map, gray_map, cv::COLOR_BGR2GRAY);
        

        nav_msgs::msg::OccupancyGrid grid;
        rclcpp::Time now = this->get_clock()->now();
    
        grid.header.stamp = now;
        grid.info.map_load_time = now;
        grid.info.resolution = 0.05;
        grid.header.frame_id = "base_link";
        grid.info.height = height;
        grid.info.width = width;
        grid.info.origin.position.x = -centrX/20;
        grid.info.origin.position.y = -centrY/20;
        grid.info.origin.position.z = 0.0;

        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(gray_map.data, gray_map.data + gray_map.total());
        //std::cout << "OK" << std::endl;
        publisher_->publish(grid);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
     
};



int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMap>());
  rclcpp::shutdown();
  return 0;
}