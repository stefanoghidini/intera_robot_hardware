#pragma once

#include <itia_basic_hardware_interface/itia_basic_hardware_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <itia_basic_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <name_sorting/name_sorting.h>
#include <mutex>
#include <intera_core_msgs/JointCommand.h>
// namespace hardware_interface
// {
//   /// \ref JointCommandInterface for commanding acceleration-based joints.
//   class AccelerationJointInterface : public JointCommandInterface {};
// }



namespace itia_hardware_interface
{
  
  class InteraRobotHW: public itia_hardware_interface::BasicRobotHW
  {
  public:
    InteraRobotHW(std::vector<std::string> joint_names);
    virtual ~InteraRobotHW()
    {
      m_mutex.lock();
      m_mutex.unlock();
    };
    virtual void shutdown();
    virtual void read(const ros::Time& time, const ros::Duration& period);
    virtual void write(const ros::Time& time, const ros::Duration& period);
    
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) ;
    
    virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) ;
    
    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& stop_list);
    
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list);
    
    

    
  protected:
    virtual void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    
    std::shared_ptr<ros::Subscriber> m_js_sub;
    std::shared_ptr<ros::Publisher>  m_js_pub;
    std::shared_ptr<ros::Publisher>  m_cmd_pub;
    sensor_msgs::JointStatePtr m_msg;
    intera_core_msgs::JointCommandPtr m_cmd_msg;

    hardware_interface::JointStateInterface    m_js_jh; //interface for reading joint state
    hardware_interface::PositionJointInterface m_p_jh; //interface for writing position target
    hardware_interface::VelocityJointInterface m_v_jh; //interface for writing velocity target
    hardware_interface::EffortJointInterface   m_e_jh; //interface for writing effort target
    hardware_interface::PosVelEffJointInterface m_pve_jh;
    hardware_interface::VelEffJointInterface m_ve_jh;
    
    
    bool m_p_jh_active;
    bool m_v_jh_active;
    bool m_e_jh_active;
    
    std::vector<std::string> m_joint_names;
    
    
    std::vector<double> m_pos; // feedback position
    std::vector<double> m_vel; // feedback velocity
    std::vector<double> m_eff; // feedback effort
    
    std::vector<double> m_cmd_pos; //target position
    std::vector<double> m_cmd_vel; //target velocity
    std::vector<double> m_cmd_eff; //target effort
    
    
    
    unsigned int m_nAx;
    unsigned int m_missing_messages;
    unsigned int m_max_missing_messages;
    bool m_topic_received;
    
    ros::Time m_start_time;
    std::mutex m_mutex;
    
    enum status { created, initialized, run, error };
    
  };
}
