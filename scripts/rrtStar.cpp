#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <stdio.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
 
class MinimalPublisher : public rclcpp::Node
{
  public:

    
    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {

      while(true){
      counter();
      }
      /*
      // Publisher publishes String messages to a topic named "addison". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<std_msgs::msg::String>("addison",10);
       
      // Initialize the timer. The timer_callback function will execute every
      // 500 milliseconds.
      timer_ = this->create_wall_timer(
      10000ms, std::bind(&MinimalPublisher::timer_callback, this));
      */
    }
 
  private:

    void counter(){
      auto message = std_msgs::msg::String();      
      // Set our message's data attribute and increment the message count by 1
      message.data = "Hi Automatic Addison! " + std::to_string(count_++);

      RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
    }


    // This method executes every 500 milliseconds
    void timer_callback()
    {
      // Create a new message of type String
      auto message = std_msgs::msg::String();
       
      // Set our message's data attribute and increment the message count by 1
      message.data = "Hi Automatic Addison! " + std::to_string(count_++);
 
      // Print every message to the terminal window      
      RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
       
      // Publish the message to the topic named "addison"
      publisher_->publish(message);
    
    }
     
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
  
    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   
    // Declaration of the count_ attribute
    int count_;
};



int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node  = rclcpp::Node::make_shared("Fist");

  rclcpp::spin(std::make_shared<MinimalPublisher>());

  rclcpp::shutdown();
  return 0;
}