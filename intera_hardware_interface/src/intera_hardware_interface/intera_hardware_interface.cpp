#include <intera_hardware_interface/intera_hardware_interface.h>



namespace itia_hardware_interface
{
  
InteraRobotHW::InteraRobotHW(std::vector< std::string > joint_names)
{
  m_joint_names=joint_names;
  m_nAx=m_joint_names.size();
  
  if (m_nAx==0)
    ROS_WARN("[%s] no joints selected",m_robot_hw_nh.getNamespace().c_str());
  
}

bool InteraRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!itia_hardware_interface::BasicRobotHW::init(root_nh, robot_hw_nh))
  {
    ROS_ERROR("[%s] BasicRobotHW error",robot_hw_nh.getNamespace().c_str());
    return false;
  }
  
  std::string read_js_topic;
  if (!m_robot_hw_nh.getParam("feedback_joint_state_topic",read_js_topic))
  {
    ROS_ERROR("[%s] feedback_joint_state_topic not defined",m_robot_hw_nh.getNamespace().c_str());
    m_status=itia_hardware_interface::with_error;
    
    diagnostic_msgs::DiagnosticStatus diag;
    diag.name=robot_hw_nh.getNamespace();
    diag.hardware_id=robot_hw_nh.getNamespace();
    diag.level=diagnostic_msgs::DiagnosticStatus::ERROR;
    diag.message="Hardware interface "+robot_hw_nh.getNamespace()+"not initialized, no feeback joint topic";
    m_diagnostic.status.push_back(diag);
    
    return false;
  }
  
  std::string write_js_topic;
  if (!m_robot_hw_nh.getParam("command_joint_state_topic",write_js_topic))
  {
    ROS_ERROR("[%s] command_joint_state_topic not defined",robot_hw_nh.getNamespace().c_str());
    m_status=with_error;
    
    diagnostic_msgs::DiagnosticStatus diag;
    diag.name=robot_hw_nh.getNamespace();
    diag.hardware_id=robot_hw_nh.getNamespace();
    diag.level=diagnostic_msgs::DiagnosticStatus::ERROR;
    diag.message="Hardware interface "+robot_hw_nh.getNamespace()+"not initialized, no feeback joint topic";
    m_diagnostic.status.push_back(diag);
    
    return false;
  }
  
  int tmp;
  if (!m_robot_hw_nh.getParam("maximum_missing_messages",tmp))
  {
    ROS_INFO("[%s] maximum_missing_messages not defined, set equal to 50",robot_hw_nh.getNamespace().c_str());
    tmp=50;
  }
  m_max_missing_messages=tmp;
  
  m_pos.resize(m_nAx);
  m_vel.resize(m_nAx);
  m_eff.resize(m_nAx);
  
  std::fill(m_pos.begin(),m_pos.end(),0.0);
  std::fill(m_vel.begin(),m_vel.end(),0.0);
  std::fill(m_eff.begin(),m_eff.end(),0.0);
  
  m_topic_received=false;
  m_js_sub.reset(new ros::Subscriber());
  m_js_pub.reset(new ros::Publisher());
  m_cmd_pub.reset(new ros::Publisher());

  *m_js_sub= ros::Subscriber(m_robot_hw_nh.subscribe<sensor_msgs::JointState>(read_js_topic,1,&itia_hardware_interface::InteraRobotHW::jointStateCallback,this));
  *m_js_pub= ros::Publisher(m_robot_hw_nh.advertise<sensor_msgs::JointState>(write_js_topic+"_log",1));
  *m_cmd_pub= ros::Publisher(m_robot_hw_nh.advertise<intera_core_msgs::JointCommand>(write_js_topic,1));

  
  double timeout=10;
  if (!m_robot_hw_nh.getParam("feedback_joint_state_timeout",timeout))
  {
    ROS_INFO("[%s] feedback_joint_state_timeout not defined, set equal to 10",robot_hw_nh.getNamespace().c_str());
    timeout=10;
  }
  

  ros::Time t0 = ros::Time::now();
  while (!m_topic_received && (ros::Time::now() - t0).toSec() <timeout )
  {
    m_queue.callAvailable();
    ros::Duration(0.001).sleep();
  }
  
  if (!m_topic_received)
  {
    ROS_ERROR("[%s] feedback_joint_state_topic not received in %f seconds",robot_hw_nh.getNamespace().c_str(),timeout);
    m_status=with_error;
    diagnostic_msgs::DiagnosticStatus diag;
    diag.name=robot_hw_nh.getNamespace();
    diag.hardware_id=robot_hw_nh.getNamespace();
    diag.level=diagnostic_msgs::DiagnosticStatus::ERROR;
    diag.message="Hardware interface "+robot_hw_nh.getNamespace()+"not initialized, no feeback messages received";
    m_diagnostic.status.push_back(diag);
    
    return false;    
  }
  
  m_cmd_pos.resize(m_nAx);
  m_cmd_vel.resize(m_nAx);
  m_cmd_eff.resize(m_nAx);
  
  m_cmd_pos=m_pos;
  m_cmd_vel=m_vel;
  m_cmd_pos=m_eff;
  

  
  for (std::string& joint_name: m_joint_names) 
  {
    
    auto i = &joint_name-&m_joint_names[0];
    
    hardware_interface::JointStateHandle state_handle(joint_name, 
                                                      &(m_pos.at(i)), 
                                                      &(m_vel.at(i)), 
                                                      &(m_eff.at(i)));
    
    
    m_js_jh.registerHandle(state_handle);
    
    m_p_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_pos.at(i))) );
    m_v_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_vel.at(i))) );
    m_e_jh.registerHandle( hardware_interface::JointHandle(state_handle, &(m_cmd_eff.at(i))) );
    
    m_pve_jh.registerHandle(hardware_interface::PosVelEffJointHandle(state_handle,&(m_cmd_pos.at(i)),&(m_cmd_vel.at(i)),&(m_cmd_eff.at(i))));
    m_ve_jh.registerHandle(hardware_interface::VelEffJointHandle(state_handle,&(m_cmd_vel.at(i)),&(m_cmd_eff.at(i))));
  }
  
  
  
  registerInterface(&m_js_jh);
  registerInterface(&m_p_jh);
  registerInterface(&m_v_jh);
  registerInterface(&m_e_jh);
  registerInterface(&m_pve_jh);
  registerInterface(&m_ve_jh);
  
  m_p_jh_active=m_v_jh_active=m_e_jh_active=false;
  
  m_msg.reset(new sensor_msgs::JointState());
  m_msg->name=m_joint_names;
  m_msg->position=m_pos;
  m_msg->velocity=m_vel;
  m_msg->effort.resize(m_nAx,0);

  m_cmd_msg.reset(new intera_core_msgs::JointCommand());
  m_cmd_msg->names=m_joint_names;
  m_cmd_msg->mode=m_cmd_msg->POSITION_MODE;
  m_cmd_msg->position=m_pos;
  m_cmd_msg->velocity=m_vel;
  m_cmd_msg->effort.resize(m_nAx,0);

  m_start_time=ros::Time::now();
  return true;
}

void InteraRobotHW::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  
  m_mutex.lock();
  std::vector<std::string> names=msg->name;
  std::vector<double> pos=msg->position;
  std::vector<double> vel=msg->velocity;
  std::vector<double> eff=msg->effort;
  
  if ( (pos.size()<m_nAx) || (vel.size()<m_nAx) || (eff.size()<m_nAx) || (names.size()<m_nAx))
  {
    ROS_FATAL("Dimension are wrong: pos %zu, vel %zu, eff %zu, names %zu, expected %u",pos.size(),vel.size(),eff.size(),names.size(),m_nAx);
    m_topic_received=false;
    return;
  }
    
//  for (unsigned int i=0; i<m_joint_names.size();i++)
//  {
//    std::cout << names.at(i).c_str() << std::endl;
//    std::cout << pos.at(i) << std::endl;
//    std::cout << m_joint_names.at(i).c_str() << std::endl;
//  }
//  std::cout << "---" << std::endl;

//  ROS_ERROR("QUI");
  if (!name_sorting::permutationName(m_joint_names,names,pos,vel,eff))
  {
    m_topic_received=false;
    ROS_WARN_THROTTLE(0.1,"[%s] feedback joint states names are wrong!",m_robot_hw_nh.getNamespace().c_str());
    return;
  }
//  ROS_ERROR("QUI");

  m_topic_received=true;
  for (unsigned int idx=0;idx<m_nAx;idx++)
  {
    m_pos.at(idx)=pos.at(idx);
    m_vel.at(idx)=vel.at(idx);
    m_eff.at(idx)=eff.at(idx);
  }
  m_mutex.unlock();

}

void InteraRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();

  if ((!m_topic_received) && ((time-m_start_time).toSec()>0.1))
    m_missing_messages++;
  else
    m_missing_messages=0;

  m_topic_received=false;
  
  if (m_missing_messages>m_max_missing_messages)
  {
    ROS_ERROR("[%s] maximum_missing_messages (%d) reached ",m_robot_hw_nh.getNamespace().c_str(),m_missing_messages);
    m_status=with_error;
    diagnostic_msgs::DiagnosticStatus diag;
    diag.name=m_robot_hw_nh.getNamespace();
    diag.hardware_id=m_robot_hw_nh.getNamespace();
    diag.level=diagnostic_msgs::DiagnosticStatus::ERROR;
    diag.message="Hardware interface "+m_robot_hw_nh.getNamespace()+" run time: no feeback messages received";
    m_diagnostic.status.push_back(diag);
  }
  
}

void InteraRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  
  if (!m_p_jh_active && !m_v_jh_active && !m_e_jh_active)
  {
    return;
  }
  
  
  if (m_p_jh_active)
  {
    m_msg->position = m_cmd_pos;
  }
  else
  {
    m_msg->position.resize(m_nAx);
    std::fill(m_msg->position.begin(),m_msg->position.end(),0.0);
  }
  
  if (m_v_jh_active)
    m_msg->velocity = m_cmd_vel;
  else
  {
    m_msg->velocity.resize(m_nAx);
    std::fill(m_msg->velocity.begin(),m_msg->velocity.end(),0.0);
  }
  
  if (m_e_jh_active)
    m_msg->effort   = m_cmd_eff;
  else
  {
    m_msg->effort.resize(m_nAx);
    std::fill(m_msg->effort.begin(),m_msg->effort.end(),0.0);
  }
  m_msg->name=m_joint_names;
  m_msg->header.stamp=ros::Time::now();

  m_cmd_msg->names=m_msg->name;
  m_cmd_msg->position=m_msg->position;
  m_cmd_msg->velocity=m_msg->velocity;
  m_cmd_msg->acceleration.resize(m_msg->position.size(),0);
  m_cmd_msg->effort.resize(m_msg->position.size(),0);
  m_cmd_msg->mode=m_cmd_msg->POSITION_MODE;
  m_cmd_msg->header.stamp=ros::Time::now();

  m_mutex.lock();
  m_js_pub->publish(m_msg);
  m_cmd_pub->publish(m_cmd_msg);
  m_mutex.unlock();
  
  sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState());
  m_msg.swap(msg);

  intera_core_msgs::JointCommandPtr cmd_msg(new intera_core_msgs::JointCommand());
  m_cmd_msg.swap(cmd_msg);

}

bool InteraRobotHW::prepareSwitch(const std::list< hardware_interface::ControllerInfo >& start_list,
                                 const std::list< hardware_interface::ControllerInfo >& stop_list)
{
  bool p_jh_active, v_jh_active, e_jh_active;
  p_jh_active=v_jh_active=e_jh_active=false;
  
  p_jh_active=m_p_jh_active;
  v_jh_active=m_v_jh_active;
  e_jh_active=m_e_jh_active;

  for (const hardware_interface::ControllerInfo& controller: stop_list)
  {
    for (const hardware_interface::InterfaceResources& res: controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active=false;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active=false;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active=false;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active=false;
        e_jh_active=false;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active=false;
        v_jh_active=false;
        e_jh_active=false;
      }
    }
  }
  for (const hardware_interface::ControllerInfo& controller: start_list)
  {
    
    for (const hardware_interface::InterfaceResources& res: controller.claimed_resources)
    {
      if (!res.hardware_interface.compare("hardware_interface::PositionJointInterface"))
        p_jh_active=true;
      if (!res.hardware_interface.compare("hardware_interface::VelocityJointInterface"))
        v_jh_active=true;
      if (!res.hardware_interface.compare("hardware_interface::EffortJointInterface"))
        e_jh_active=true;
      if (!res.hardware_interface.compare("hardware_interface::VelEffJointInterface"))
      {
        v_jh_active=true;
        e_jh_active=true;
      }
      if (!res.hardware_interface.compare("hardware_interface::PosVelEffJointInterface"))
      {
        p_jh_active=true;
        v_jh_active=true;
        e_jh_active=true;
      }
    }
  }
  
  m_p_jh_active=p_jh_active;
  m_v_jh_active=v_jh_active;
  m_e_jh_active=e_jh_active;
  return true;
}

void InteraRobotHW::doSwitch(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list)
{
}

bool InteraRobotHW::checkForConflict(const std::list< hardware_interface::ControllerInfo >& info)
{
  // Each controller can use more than a hardware_interface for a single joint (for example: position, velocity, effort). 
  // One controller can control more than one joint.
  // A joint can be used only by a controller.
  
  std::vector<bool> global_joint_used(m_nAx); 
  std::fill(global_joint_used.begin(),global_joint_used.end(),false);
  
  for (hardware_interface::ControllerInfo controller: info)
  {
    std::vector<bool> single_controller_joint_used(m_nAx); 
    std::fill(single_controller_joint_used.begin(),single_controller_joint_used.end(),false);
    
    for (hardware_interface::InterfaceResources res: controller.claimed_resources)
    {
      for (std::string name: res.resources)
      {
        for (unsigned int iJ=0;iJ<m_nAx;iJ++)
        {
          if (!name.compare(m_joint_names.at(iJ)))
          {
            if (global_joint_used.at(iJ)) // if already used by another
            {
              ROS_ERROR("Joint %s is already used by another controller",name.c_str());
              diagnostic_msgs::DiagnosticStatus diag;
              diag.name=m_robot_hw_nh.getNamespace();
              diag.hardware_id=m_robot_hw_nh.getNamespace();
              diag.level=diagnostic_msgs::DiagnosticStatus::ERROR;
              diag.message="Hardware interface "+m_robot_hw_nh.getNamespace()+" run time: Joint " + name + " is already used by another controller";
              m_diagnostic.status.push_back(diag);
              
              return true;
            }
            else
              single_controller_joint_used.at(iJ);
          }
        }
      }
    }
    for (unsigned int iJ=0;iJ<m_nAx;iJ++)
      global_joint_used.at(iJ)= global_joint_used.at(iJ) || single_controller_joint_used.at(iJ);
    
  }
  return false;
}

void InteraRobotHW::shutdown()
{
  m_js_pub.reset();
  m_cmd_pub.reset();
}


}

