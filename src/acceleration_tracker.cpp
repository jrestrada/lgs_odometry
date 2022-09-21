#include <vector>
#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
using std::placeholders::_1;
using IMUMSG = sensor_msgs::msg::Imu;
using Time = std::chrono::high_resolution_clock;
using Ms = std::chrono::milliseconds;
class MinimalSubscriber : public rclcpp::Node{
  public:
    MinimalSubscriber(): Node("minimal_subscriber"){
      subscription_ = this->create_subscription<IMUMSG>("Imu", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
  private:
    void topic_callback(const IMUMSG & msg) {
        if(msg.linear_acceleration.x != 0.0){
          beginTrack(msg.linear_acceleration.x);
        } else {
          if (!atRest) endTrack();
        }
        if (atRest) t1 = Time::now();
    }
    void beginTrack(double a){
      atRest = false;
      t2 = Time::now(); 
      Ms d = std::chrono::duration_cast<Ms>(t2-t1);
      ds = (float)d.count()/1000;
      accel.push_back(a);
      veloc.push_back(veloc.back()+accel.back()*ds);     
      veloc.push_back(posit.back()+veloc.back()*ds);      
      RCLCPP_INFO(this->get_logger(), "Time interval: '%f'", ds);
      t1 = t2;
    }
    void endTrack(){
      RCLCPP_INFO(this->get_logger(), "Crawler moved %f ", posit.end());
      accel.erase(accel.begin()+1,accel.end());
      veloc.erase(veloc.begin()+1,veloc.end());
      posit.erase(posit.begin()+1,posit.end());
      atRest = true;
    }
    float ds;
    std::vector<float> accel {0.0};
    std::vector<float> veloc {0.0};
    std::vector<float> posit {0.0};
    bool atRest = true;
    std::chrono::time_point<Time> t1, t2;
    rclcpp::Subscription<IMUMSG>::SharedPtr subscription_;
};
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}