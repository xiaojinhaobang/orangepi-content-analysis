#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <chrono>
#include <vector>
#include <iostream>
class UartToStm32 {
public:
 rclcpp::Node::SharedPtr node_=std::make_shared<rclcpp::Node>("callback_check");
 rclcpp::TimerBase::SharedPtr a2_ready_timer_;
 unsigned a2_ready_remaining_=0, sent=0;
 void sendA2ReadyResponse(){++sent;}
 void activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg);
};
void UartToStm32::activeControllerCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data != 2 || a2_ready_remaining_ > 0) {
    return;
  }
  sendA2ReadyResponse();
  a2_ready_remaining_ = 2;
  if (!a2_ready_timer_) {
    a2_ready_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      sendA2ReadyResponse();
      if (--a2_ready_remaining_ == 0) {
        a2_ready_timer_->cancel();
      }
    });
  } else {
    a2_ready_timer_->reset();
  }
}


int main(int argc,char**argv){
 rclcpp::init(argc,argv); UartToStm32 x;
 using Clock=std::chrono::steady_clock;
 auto start=Clock::now(),last=start; double maxgap=0; unsigned pulses=0;
 auto tick=x.node_->create_wall_timer(std::chrono::milliseconds(20),[&](){
  auto now=Clock::now(); maxgap=std::max(maxgap,std::chrono::duration<double>(now-last).count()); last=now; ++pulses;
 });
 auto msg=std::make_shared<std_msgs::msg::UInt8>(); msg->data=2;
 for(int i=0;i<10;++i) x.activeControllerCallback(msg);
 bool again=false;
 while(std::chrono::duration<double>(Clock::now()-start).count()<2.7){
  rclcpp::spin_some(x.node_);
  if(!again && std::chrono::duration<double>(Clock::now()-start).count()>2.0){x.activeControllerCallback(msg); again=true;}
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }
 bool pass=x.sent==6 && pulses>120 && maxgap<0.1;
 std::cout<<"sent="<<x.sent<<" pulses="<<pulses<<" max_gap_s="<<maxgap<<" result="<<(pass?"PASS":"FAIL")<<std::endl;
 rclcpp::shutdown(); return pass?0:1;
}
