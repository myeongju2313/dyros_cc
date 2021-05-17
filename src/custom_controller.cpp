 #include "custom_controller.h"
#include <fstream>

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_)
{
    ControlVal_.setZero();
    pedal_command = dc_.nh.subscribe("/tocabi/pedalcommand", 100, &CustomController::PedalCommandCallback, this);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}
void CustomController::taskCommandToCC(TaskCommand tc_)
{
    tc = tc_;
}

void CustomController::PedalCommandCallback(const dyros_pedal::WalkingCommandConstPtr &msg){

  if(joy_input_enable_ == true){
    joystick_input(0) = msg->step_length_x; //FW
    joystick_input(2) = msg->theta;
    joystick_input(3) = msg->z; //BW
    joystick_input(1) = (joystick_input(0) + 1)/2 + abs(joystick_input(2)) + (joystick_input(3) + 1)/2;
    
  }
  else{
    joystick_input(1) = - 1.0;
  }

  if(joystick_input(1) > 0){
    walking_enable_ = true;
    walking_end_flag = 1;
  }
}

ofstream MJ_graph("/home/dyros/data/myeongju/MJ_graph.txt");
ofstream MJ_joint("/home/dyros/data/myeongju/MJ_joint.txt");
ofstream MJ_ZMP("/home/dyros/data/myeongju/MJ_zmp.txt");
// ofstream MJ_graph("/home/myeongju/MJ_graph.txt");
// ofstream MJ_joint("/home/myeongju/MJ_joint.txt");
// ofstream MJ_ZMP("/home/myeongju/MJ_zmp.txt");

void CustomController::computeSlow()
{
    if (tc.mode == 10)
    {
        if(initial_flag == 0)
        {   
            Joint_gain_set_MJ();
            walking_enable_ = true;            
            // Initial pose             
            ref_q_ = rd_.q_;
            for(int i = 0; i < 12; i ++)
            {
              Initial_ref_q_(i) = ref_q_(i);
            }
            initial_flag = 1; 
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            cout << "mode = 10" << endl;
        }

        wbc_.set_contact(rd_, 1, 1);  
        Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);
          
        for(int i = 0; i < MODEL_DOF; i++)
        { ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i) ; }
       
    }
    else if (tc.mode == 11)
    {
      if(walking_enable_ == true)
      {
        if(walking_tick_mj == 0)
        {     
            parameterSetting();
            cout << "parameter setting OK" << endl;
            cout << "mode = 11" << endl;
        }        
        updateInitialState();
        getRobotState();
        floatToSupportFootstep();

        if(current_step_num_< total_step_num_)
        {   
            getZmpTrajectory();
            getComTrajectory();            
            getFootTrajectory();
            getPelvTrajectory();
            supportToFloatPattern();
            computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des);
            
            Compliant_control(q_des);
            for(int i = 0; i < 12; i ++)
            {
              //ref_q_(i) = q_des(i);
              ref_q_(i) = DOB_IK_output_(i);
            }            
            hip_compensator();
            GravityCalculate_MJ();

            if(walking_tick_mj < 1.0*hz_)
            {
              for(int i = 0; i < 12; i ++)
              { ref_q_(i) = DyrosMath::cubic(walking_tick_mj, 0, 1.0*hz_, Initial_ref_q_(i), q_des(i), 0.0, 0.0); }
            }

            CP_compen_MJ();
            
            for(int i = 0; i < MODEL_DOF; i++)
            {
              ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i) + Tau_CP(i) ;  
              // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
            }               
                        
            desired_q_not_compensated_ = ref_q_;           

            updateNextStepTime();

            q_prev_MJ_ = rd_.q_;            
        }        
      }
      else
      {
        if(walking_end_flag == 0)
        {
          cout << "walking finish" << endl;
          walking_end_flag = 1; initial_flag = 0;        
        } 

        wbc_.set_contact(rd_, 1, 1);
        Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);
        for(int i = 0; i < MODEL_DOF; i++)
        { ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i); }
      }        
 
    }
    else if (tc.mode == 12)
    {
        if(initial_flag == 0)
        {
          Joint_gain_set_MJ();
          walking_enable_ = false;                        
          ref_q_ = rd_.q_;
          for(int i = 0; i < 12; i ++)
          {
            Initial_ref_q_(i) = ref_q_(i);
          }
          initial_flag = 1; 
          q_prev_MJ_ = rd_.q_;
          walking_tick_mj = 0;
          walking_end_flag = 1;
          joy_input_enable_ = true;
          cout << "mode = 12 : Pedal Init" << endl;
        }

        wbc_.set_contact(rd_, 1, 1);  
        Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);
          
        for(int i = 0; i < MODEL_DOF; i++)
        { ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i) ; }
       
    }
    else if (tc.mode == 13)
    {
      if(walking_enable_ == true)
      {
        if(walking_tick_mj == 0)
        {     
            parameterSetting();
            cout <<  "\n\n\n\n" << endl;
            cout <<  "___________________________ " << endl;
            cout <<  "\n           Start " << endl;
            cout << "parameter setting OK" << endl;
            cout << "mode = 13" << endl;
        }        
        updateInitialStateJoy();
        getRobotState();
        floatToSupportFootstep();

        if(current_step_num_< total_step_num_)
        {   
            getZmpTrajectory();
            getComTrajectory();            
            getFootTrajectory();
            getPelvTrajectory();
            supportToFloatPattern();
            computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des);
            
            Compliant_control(q_des);
            for(int i = 0; i < 12; i ++)
            {
              //ref_q_(i) = q_des(i);
              ref_q_(i) = DOB_IK_output_(i);
            }            
            hip_compensator();
            GravityCalculate_MJ();

            if(walking_tick_mj < 1.0*hz_)
            {
              for(int i = 0; i < 12; i ++)
              { ref_q_(i) = DyrosMath::cubic(walking_tick_mj, 0, 1.0*hz_, Initial_ref_q_(i), q_des(i), 0.0, 0.0); }
            }

            CP_compen_MJ();
            
            for(int i = 0; i < MODEL_DOF; i++)
            {
              ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i) + Tau_CP(i) ;  
              // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
            }               
                        
            desired_q_not_compensated_ = ref_q_;           

            updateNextStepTimeJoy();

            q_prev_MJ_ = rd_.q_;            
        }        
      }
      else
      {
        if(walking_end_flag == 0)
        {
          cout << "walking finish" << endl;
          walking_end_flag = 1; initial_flag = 0;        
        } 

        wbc_.set_contact(rd_, 1, 1);
        Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);
        for(int i = 0; i < MODEL_DOF; i++)
        { ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i); }
      }        
 
    }   
}

void CustomController::computeFast()
{
    if (tc.mode == 10)
    {
    }
    else if (tc.mode == 11)
    {
    }
}

void CustomController::updateInitialState()
{
  if(walking_tick_mj == 0)
  {
    //calculateFootStepTotal();
    calculateFootStepTotal_MJ();
        
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].Rotm); //ZYX multiply
        
    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));   
    
    pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].Rotm;

    pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
    //pelv_float_init_.translation()(0) += 0.11;
    
    pelv_float_init_.translation()(0) = 0;
    pelv_float_init_.translation()(1) = 0;

    lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].Rotm;
    lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
    
    lfoot_float_init_.translation()(0) = 0;
    lfoot_float_init_.translation()(1) = 0.1225;

    rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].Rotm;
    rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
    
    rfoot_float_init_.translation()(0) = 0;
    rfoot_float_init_.translation()(1) = -0.1225;

    com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
    
    com_float_init_(0) = 0;
    com_float_init_(1) = 0;

    if(aa == 0)
    {
      lfoot_float_init_.translation()(1) = 0.1025;
      rfoot_float_init_.translation()(1) = -0.1025;
      aa = 1;
    }
    cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_(2)*180/3.141592 << endl;
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(0, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(0, 6) == 1)
    { ref_frame = lfoot_float_init_; }
         
    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
    com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
     
    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();
     
    if(foot_step_(0,6) == 1) //left suppport foot
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    else
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }

    pelv_support_start_ = pelv_support_init_;
    total_step_num_ = foot_step_.col(1).size();
    
    xi_ = com_support_init_(0); // preview parameter
    yi_ = com_support_init_(1);
    zc_ = com_support_init_(2);
    
  }
  else if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change 
  { 
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].Rotm); //ZYX multiply

    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    
    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].Rotm);  
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].Rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));
    
    pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].Rotm;

    pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
    
    lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].Rotm;
    //lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].Rotm;
    lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
    
    rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].Rotm;
    //rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].Rotm;
    rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
    
    com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { ref_frame = lfoot_float_init_; }     

    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
    com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
      
  }
}

void CustomController::getRobotState()
{
  pelv_rpy_current_.setZero();
  pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].Rotm); //ZYX multiply

  R_angle = pelv_rpy_current_(0);
  P_angle = pelv_rpy_current_(1); 
  pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    
  rfoot_rpy_current_.setZero();
  lfoot_rpy_current_.setZero();
  rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].Rotm);  
  lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].Rotm);

  rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
  lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
  rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
  lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

  pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].Rotm;

  pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);

  lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].Rotm;
  //lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].Rotm;
  lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
  
  rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].Rotm;
  //rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].Rotm;
  rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
   
  com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
  com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].v); 
  
  if(walking_tick_mj == 0)
  { com_float_current_dot_LPF = com_float_current_dot; com_float_current_dot_prev = com_float_current_dot; }

  com_float_current_dot_prev = com_float_current_dot;
  com_float_current_dot_LPF = 1/(1+2*M_PI*6.0*del_t)*com_float_current_dot_LPF + (2*M_PI*6.0*del_t)/(1+2*M_PI*6.0*del_t)*com_float_current_dot; 

  if(walking_tick_mj == 0)
  { com_float_current_LPF = com_float_current_; }
  
  com_float_current_LPF = 1/(1+2*M_PI*8.0*del_t)*com_float_current_LPF + (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*com_float_current_;    

  if(foot_step_(current_step_num_, 6) == 0)
  { supportfoot_float_current_ = rfoot_float_current_; }
  else if(foot_step_(current_step_num_, 6) == 1)
  { supportfoot_float_current_ = lfoot_float_current_; }
  
  pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * pelv_float_current_;   
  lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * lfoot_float_current_;
  rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * rfoot_float_current_;
    
  //cout << "L : " << lfoot_float_current_.linear() << "\n" <<  "R : " << rfoot_float_current_.linear() << endl; 
  com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_), com_float_current_);  
  com_support_current_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_), com_float_current_LPF);

  l_ft_ = rd_.ContactForce_FT_raw.segment(0, 6);
  r_ft_ = rd_.ContactForce_FT_raw.segment(6, 6);    

  Eigen::Vector2d left_zmp, right_zmp;

  left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_support_current_.translation()(0);
  left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_support_current_.translation()(1);

  right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_support_current_.translation()(0);
  right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_support_current_.translation()(1);

  zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2)); // ZMP X
  zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2)); // ZMP Y
  
  wn = sqrt(GRAVITY/zc_);
  
  if(walking_tick_mj == 0)
  { zmp_measured_LPF_.setZero(); }

  zmp_measured_LPF_ = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*zmp_measured_ + 1/(1+2*M_PI*8.0*del_t)*zmp_measured_LPF_;
 
}

void CustomController::calculateFootStepTotal()
{
  double initial_rot = 0.0;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot = atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 5*DEG2RAD;
  else
    initial_drot = -5*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot - initial_total_step_number*initial_drot;

  final_rot = target_theta_ - initial_rot;
  if(final_rot > 0.0)
    final_drot = 5*DEG2RAD;
  else
    final_drot = -5*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot - final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_ + target_y_*target_y_);
  double dlength = step_length_x_;
  unsigned int middle_total_step_number = length_to_target/dlength;
  double middle_residual_length = length_to_target - middle_total_step_number*dlength;

  if(length_to_target == 0)
  {
    middle_total_step_number = 30; //
    dlength = 0;
  }

  unsigned int number_of_foot_step;

  int del_size;

  del_size = 1;
  number_of_foot_step = initial_total_step_number*del_size + middle_total_step_number*del_size + final_total_step_number*del_size;

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    if(initial_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    if(abs(final_residual_angle) >= 0.0001)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
      number_of_foot_step = number_of_foot_step + del_size;
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right;
  temp2 = -is_right;
  temp3 = -is_right;


  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
  {
    for (int i =0 ; i < initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*0.1025*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*0.1025*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index++;
    }

    if(temp == is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.1025*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1025*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.1025*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1025*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.1025*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1025*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.1025*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.1025*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;
      }
    }
    else if(temp == -is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.1025*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.1025*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;

      temp *= -1;

      foot_step_(index,0) = temp*0.1025*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.1025*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
  {
    for (int i = 0 ; i < middle_total_step_number; i++)
    {
      temp2 *= -1;
      
      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1)) + temp2*sin(initial_rot)*(0.1025);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1)) - temp2*cos(initial_rot)*(0.1025);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index ++;
    }

    if(temp2 == is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;

        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
    }
    else if(temp2 == -is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;
    }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    for(int i = 0 ; i < final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1025*sin((i+1)*final_drot + initial_rot);
      foot_step_(index,1) = final_position_y - temp3*0.1025*cos((i+1)*final_drot + initial_rot);
      foot_step_(index,5) = (i+1)*final_drot + initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1025*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.1025*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1025*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.1025*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.1025*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.1025*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
  }
}

void CustomController::calculateFootStepTotal_MJ()
{
  double initial_rot = 0.0;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot = atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot - initial_total_step_number*initial_drot;

  final_rot = target_theta_ - initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot - final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_ + target_y_*target_y_);
  double dlength = step_length_x_;
  unsigned int middle_total_step_number = length_to_target/dlength;
  double middle_residual_length = length_to_target - middle_total_step_number*dlength;

  double step_width_init;
  double step_width;

  step_width_init = 0.01;   
  step_width = 0.02;
 
  if(length_to_target == 0)
  {
    middle_total_step_number = 10; //
    dlength = 0;
  }

  unsigned int number_of_foot_step;

  int del_size;

  del_size = 1;
  number_of_foot_step = 2 + initial_total_step_number*del_size + middle_total_step_number*del_size + final_total_step_number*del_size;

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    if(initial_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    if(abs(final_residual_angle) >= 0.0001)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
      number_of_foot_step = number_of_foot_step + del_size;
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right;
  temp2 = -is_right;
  temp3 = -is_right;

  int temp0;
  temp0 = -is_right;

  double initial_dir = 0.0;

  if(aa == 0)
  {
    for (int i = 0 ; i < 2; i++)
    {
      temp0 *= -1;

      if(i == 0)
      {
        foot_step_(index,0) = cos(initial_dir)*(0.0) + temp0*sin(initial_dir)*(0.1025 + step_width_init*(i+1));
        foot_step_(index,1) = sin(initial_dir)*(0.0) - temp0*cos(initial_dir)*(0.1025 + step_width_init*(i+1));
      }
      else if(i == 1)
      {
        foot_step_(index,0) = cos(initial_dir)*(0.0) + temp0*sin(initial_dir)*(0.1025 + step_width_init*(i+1));
        foot_step_(index,1) = sin(initial_dir)*(0.0) - temp0*cos(initial_dir)*(0.1025 + step_width_init*(i+1));
      }     
      
      foot_step_(index,5) = initial_dir;
      foot_step_(index,6) = 0.5 + 0.5*temp0;
      index ++;
    }
  }
  else if(aa == 1)
  {
    for (int i = 0 ; i < 2; i++)
    {
      temp0 *= -1;

      foot_step_(index,0) = cos(initial_dir)*(0.0) + temp0*sin(initial_dir)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_dir)*(0.0) - temp0*cos(initial_dir)*(0.1025 + step_width);
      foot_step_(index,5) = initial_dir;
      foot_step_(index,6) = 0.5 + 0.5*temp0;
      index ++;
    }
  }

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
  {
    for (int i =0 ; i < initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*(0.1025 + step_width)*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index++;
    }

    if(temp == is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;
      }
    }
    else if(temp == -is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;

      temp *= -1;

      foot_step_(index,0) = temp*(0.1025 + step_width)*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*(0.1025 + step_width)*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001) // 직진, 제자리 보행
  {
    
    for (int i = 0 ; i < middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1)) + temp2*sin(initial_rot)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1)) - temp2*cos(initial_rot)*(0.1025 + step_width);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index ++;
    }

    if(temp2 == is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;

        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
    }
    else if(temp2 == -is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.1025 + step_width);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.1025 + step_width);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;
    }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    for(int i = 0 ; i < final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin((i+1)*final_drot + initial_rot);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos((i+1)*final_drot + initial_rot);
      foot_step_(index,5) = (i+1)*final_drot + initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*(0.1025 + step_width)*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*(0.1025 + step_width)*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
  }
}


void CustomController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;

    if(current_step_num_ == 0)
    {
        if(foot_step_(0,6) == 0)
        {
          reference.translation() = rfoot_float_init_.translation();
          reference.translation()(2) = 0.0;
          reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
          reference.translation()(0) = 0.0;
        }
        else
        {
          reference.translation() = lfoot_float_init_.translation();
          reference.translation()(2) = 0.0;
          reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
          reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
        for(int i=0 ;i<3; i++)
        { reference.translation()(i) = foot_step_(current_step_num_-1,i); }        
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    for(int i = 0; i < total_step_num_; i++)
    {
        for(int j = 0; j < 3; j ++)
        {temp_global_position(j) = foot_step_(i,j);}

        temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

        for(int j=0; j<3; j++)
        { foot_step_support_frame_(i,j) = temp_local_position(j); }

        foot_step_support_frame_(i,3) = foot_step_(i,3);
        foot_step_support_frame_(i,4) = foot_step_(i,4);
        if(current_step_num_ == 0)
        { foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5); }
        else
        { foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5); }
    }

    for(int j = 0; j < 3; j ++)
    temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.
 
    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());
 
    for(int j=0;j<3;j++)
      swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if(current_step_num_ == 0)
      swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
      swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
    
    for(int j=0;j<3;j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

    for(int j=0;j<3;j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if(current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
}

void CustomController::Joint_gain_set_MJ()
{
    //simulation gains
    // Kp(0) = 1800.0; Kd(0) = 70.0; // Left Hip yaw
    // Kp(1) = 2100.0; Kd(1) = 90.0;// Left Hip roll
    // Kp(2) = 2100.0; Kd(2) = 90.0;// Left Hip pitch
    // Kp(3) = 2100.0; Kd(3) = 90.0;// Left Knee pitch
    // Kp(4) = 900.0; Kd(4) = 40.0;// Left Ankle pitch
    // Kp(5) = 900.0; Kd(5) = 40.0;// Left Ankle roll

    // Kp(6) = 1800.0; Kd(6) = 70.0;// Right Hip yaw
    // Kp(7) = 2100.0; Kd(7) = 90.0;// Right Hip roll
    // Kp(8) = 2100.0; Kd(8) = 90.0;// Right Hip pitch
    // Kp(9) = 2100.0; Kd(9) = 90.0;// Right Knee pitch
    // Kp(10) = 900.0; Kd(10) = 40.0;// Right Ankle pitch
    // Kp(11) = 900.0; Kd(11) = 40.0;// Right Ankle roll

    // Kp(12) = 2200.0; Kd(12) = 90.0;// Waist yaw
    // Kp(13) = 2200.0; Kd(13) = 90.0;// Waist pitch
    // Kp(14) = 2200.0; Kd(14) = 90.0;// Waist roll
        
    // Kp(15) = 400.0; Kd(15) = 10.0;
    // Kp(16) = 800.0; Kd(16) = 10.0;
    // Kp(17) = 400.0; Kd(17) = 10.0;
    // Kp(18) = 400.0; Kd(18) = 10.0;
    // Kp(19) = 250.0; Kd(19) = 2.5;
    // Kp(20) = 250.0; Kd(20) = 2.0;
    // Kp(21) = 50.0; Kd(21) = 2.0; // Left Wrist
    // Kp(22) = 50.0; Kd(22) = 2.0; // Left Wrist
   
    // Kp(23) = 50.0; Kd(23) = 2.0; // Neck
    // Kp(24) = 50.0; Kd(24) = 2.0; // Neck

    // Kp(25) = 400.0; Kd(25) = 10.0;
    // Kp(26) = 800.0; Kd(26) = 10.0;
    // Kp(27) = 400.0; Kd(27) = 10.0;
    // Kp(28) = 400.0; Kd(28) = 10.0;
    // Kp(29) = 250.0; Kd(29) = 2.5;
    // Kp(30) = 250.0; Kd(30) = 2.0;
    // Kp(31) = 50.0; Kd(31) = 2.0; // Right Wrist
    // Kp(32) = 50.0; Kd(32) = 2.0; // Right Wrist
    
    Kp(0) = 2000.0; Kd(0) = 15.0; // Left Hip yaw
    Kp(1) = 5000.0; Kd(1) = 50.0;// Left Hip roll
    Kp(2) = 4000.0; Kd(2) = 20.0;// Left Hip pitch
    Kp(3) = 3700.0; Kd(3) = 25.0;// Left Knee pitch
    Kp(4) = 5000.0; Kd(4) = 30.0;// Left Ankle pitch /5000 / 30
    Kp(5) = 5000.0; Kd(5) = 30.0;// Left Ankle roll /5000 / 30

    Kp(6) = 2000.0; Kd(6) = 15.0;// Right Hip yaw
    Kp(7) = 5000.0; Kd(7) = 50.0;// Right Hip roll
    Kp(8) = 4000.0; Kd(8) = 20.0;// Right Hip pitch
    Kp(9) = 3700.0; Kd(9) = 25.0;// Right Knee pitch
    Kp(10) = 5000.0; Kd(10) = 30.0;// Right Ankle pitch
    Kp(11) = 5000.0; Kd(11) = 30.0;// Right Ankle roll

    Kp(12) = 6000.0; Kd(12) = 200.0;// Waist yaw
    Kp(13) = 10000.0; Kd(13) = 100.0;// Waist pitch
    Kp(14) = 10000.0; Kd(14) = 100.0;// Waist roll
        
    Kp(15) = 400.0; Kd(15) = 10.0;
    Kp(16) = 800.0; Kd(16) = 10.0;
    Kp(17) = 400.0; Kd(17) = 10.0;
    Kp(18) = 400.0; Kd(18) = 10.0;
    Kp(19) = 250.0; Kd(19) = 2.5;
    Kp(20) = 250.0; Kd(20) = 2.0;
    Kp(21) = 50.0; Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0; Kd(22) = 2.0; // Left Wrist
   
    Kp(23) = 50.0; Kd(23) = 2.0; // Neck
    Kp(24) = 50.0; Kd(24) = 2.0; // Neck

    Kp(25) = 400.0; Kd(25) = 10.0;
    Kp(26) = 800.0; Kd(26) = 10.0;
    Kp(27) = 400.0; Kd(27) = 10.0;
    Kp(28) = 400.0; Kd(28) = 10.0;
    Kp(29) = 250.0; Kd(29) = 2.5;
    Kp(30) = 250.0; Kd(30) = 2.0;
    Kp(31) = 50.0; Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0; Kd(32) = 2.0; // Right Wrist
}

void CustomController::addZmpOffset()
{
  double lfoot_zmp_offset_, rfoot_zmp_offset_;
 
  lfoot_zmp_offset_ = -0.0175;
  rfoot_zmp_offset_ = 0.0175;

  foot_step_support_frame_offset_ = foot_step_support_frame_;

  supportfoot_support_init_offset_ = supportfoot_support_init_;
 

  if(foot_step_(0,6) == 0) //right support foot
  {
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
  }
  else
  {
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
    //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
  }

  for(int i=0; i<total_step_num_; i++)
  {
    if(foot_step_(i,6) == 0)//right support, left swing
    {
      foot_step_support_frame_offset_(i,1) += lfoot_zmp_offset_;
    }
    else
    {
      foot_step_support_frame_offset_(i,1) += rfoot_zmp_offset_;
    }
  }
}

void CustomController::getZmpTrajectory()
{
  unsigned int planning_step_number = 3;
  unsigned int norm_size = 0;
 
  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_ - t_start_ + 1)*(total_step_num_ - current_step_num_) + 3.0*hz_;
  else
    norm_size = (t_last_ - t_start_ + 1)*(planning_step_number);
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_ + 1;
  addZmpOffset();  
  zmpGenerator(norm_size, planning_step_number);
}

void CustomController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
  ref_zmp_.resize(norm_size, 2);
  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;
 
  unsigned int index = 0;
  // 매 tick 마다 zmp가 3발 앞까지 계산 된다.

  if(current_step_num_ == 0) // Walking을 수행 할 때, 정지 상태 일때 3초 동안 Ref X ZMP를 0으로 보냄. Y ZMP는 제자리 유지.  
  {
    for (int i = 0; i <= t_temp_; i++) //600 tick
    {
      if(i < 1.0*hz_)
      {
        ref_zmp_(i,0) = com_support_init_(0) ;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else if(i < 2.0*hz_)
      {
        double del_x = i - 1.0*hz_;
        ref_zmp_(i,0) = com_support_init_(0) - del_x * com_support_init_(0)/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }      
      index++;
    }    
  }
  /////////////////////////////////////////////////////////////////////
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      onestepZmp(i, temp_px, temp_py);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_zmp_(index + j, 0) = temp_px(j);
        ref_zmp_(index + j, 1) = temp_py(j);    
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 3.0*hz_; j++)
    {
      ref_zmp_(index + j, 0) = ref_zmp_(index -1, 0);
      ref_zmp_(index + j, 1) = ref_zmp_(index -1, 1);
    }
    index = index + 3.0*hz_;      
  }
  else // 보행 중 사용 하는 Ref ZMP
  {
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      onestepZmp(i, temp_px, temp_py);
      for (unsigned int j = 0; j < t_total_; j++) // 1 step 보행은 1.2초, 240 tick
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
      }      
      index = index + t_total_; // 참조 zmp가 이만큼 쌓였다.      
      // 결국 실제 로봇 1Hz마다 720개의 ref_zmp를 생성함. 3.6초
    }   
  }   
}
/*
void CustomController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0;
  if(current_step_number == 0)
  {
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = -(foot_step_support_frame_offset_(current_step_number, 1))/2 ;
    B =  (supportfoot_support_init_offset_(0) + foot_step_support_frame_offset_(current_step_number, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));
        
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_px(i) = 0;
        temp_py(i) = (com_support_init_(1)) + Ky / (t_rest_init_ + t_double1_)* (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = supportfoot_support_init_offset_(0);
        temp_py(i) = supportfoot_support_init_offset_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick
      {
        temp_px(i) = B - Kx + Kx / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (supportfoot_support_init_offset_(1) + foot_step_support_frame_offset_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }    
  }
  else if(current_step_number == 1)
  {
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_offset_(current_step_number-1, 1) - supportfoot_support_init_offset_(1))/2 ;
    B = foot_step_support_frame_offset_(current_step_number-1, 0) - (supportfoot_support_init_offset_(0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-1, 0) + supportfoot_support_init_offset_(0))/2 + Kx / (t_rest_init_+ t_double1_) * (i+1);
        temp_py(i) = (foot_step_support_frame_offset_(current_step_number-1, 1) + supportfoot_support_init_offset_(1))/2 + Ky / (t_rest_init_+ t_double1_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
      {               
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-1, 0) + foot_step_support_frame_offset_(current_step_number, 0))/2 - Kx + Kx /(t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_offset_(current_step_number-1, 1) + foot_step_support_frame_offset_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }
  else
  {
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_offset_(current_step_number-1, 1) - foot_step_support_frame_offset_(current_step_number-2, 1))/2 ;
    B = foot_step_support_frame_offset_(current_step_number-1, 0) - (foot_step_support_frame_offset_(current_step_number-2, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45))) ;
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));
    for(int i = 0; i < t_total_; i++)
    {      
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-2, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 + Kx/(t_rest_init_ + t_double1_)*(i+1);
        temp_py(i) = (foot_step_support_frame_offset_(current_step_number-2, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 + Ky/(t_rest_init_ + t_double1_)*(i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0) ;
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1) ;
      }
      else if( i >= (t_total_ - t_rest_last_ - t_double2_) && (i < t_total_) && (current_step_num_ == total_step_num_ - 1))
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2;
        temp_py(i) = Ky + (foot_step_support_frame_offset_(current_step_number-1, 1) + foot_step_support_frame_offset_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }       
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_offset_(current_step_number-1, 1) + foot_step_support_frame_offset_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }  
}*/

void CustomController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
  if(current_step_number == 0)
  {
    Kx = 0;
    Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
    Kx2 = foot_step_support_frame_offset_(current_step_number,0) / 2 - supportfoot_support_init_offset_(0);
    Ky2 = foot_step_support_frame_offset_(current_step_number,1) / 2 - supportfoot_support_init_offset_(1) ;
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i+1);
        temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = 0;
        temp_py(i) = supportfoot_support_init_offset_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_ ) //1.05 ~ 1.15초 , 210 ~ 230 tick
      {
        temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }  
  }
  else if(current_step_number == 1)
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1, 0) - (foot_step_support_frame_offset_(current_step_number-1, 0) + supportfoot_support_init_(0))/2;
    Ky = foot_step_support_frame_offset_(current_step_number-1, 1) - (foot_step_support_frame_offset_(current_step_number-1, 1) + supportfoot_support_init_(1))/2;
    Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 - foot_step_support_frame_offset_(current_step_number-1, 0);
    Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 - foot_step_support_frame_offset_(current_step_number-1, 1);

    for(int i = 0; i < t_total_; i++)
    {
      if(i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_ + t_double1_) * (i+1);
        temp_py(i) = (foot_step_support_frame_offset_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_ + t_double1_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_ ) //1.05 ~ 1.2초 , 210 ~ 240 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0) + Kx2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1) + Ky2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }
  else
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1, 0) - ((foot_step_support_frame_offset_(current_step_number-2, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2);
    Ky = foot_step_support_frame_offset_(current_step_number-1, 1) - ((foot_step_support_frame_offset_(current_step_number-2, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2);
    Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 - foot_step_support_frame_offset_(current_step_number-1, 0);
    Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 - foot_step_support_frame_offset_(current_step_number-1, 1);

    for(int i = 0; i < t_total_; i++)
    {
      if(i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_offset_(current_step_number-2, 0) + foot_step_support_frame_offset_(current_step_number-1, 0))/2 + Kx/(t_rest_init_+t_double1_)*(i+1);
        temp_py(i) = (foot_step_support_frame_offset_(current_step_number-2, 1) + foot_step_support_frame_offset_(current_step_number-1, 1))/2 + Ky/(t_rest_init_+t_double1_)*(i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_ ) //1.05 ~ 1.2초 , 210 ~ 240 tick
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1, 0) + Kx2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1, 1) + Ky2/(t_double2_ + t_rest_last_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }      
    }
  }
}


void CustomController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
  { target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i); }
 
  if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
  {  
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    {
      lfoot_trajectory_support_.translation().setZero();
      lfoot_trajectory_euler_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0;
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
    }     
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      rfoot_trajectory_support_.translation().setZero();
      rfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
    }     

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
  }
 
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)  
  {
    double t_rest_temp = 0.00*hz_;
       
    if(foot_step_(current_step_num_,6) == 1)
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();             
      lfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0)
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_+ t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,0,foot_height_,0.0,0.0); }  
      else
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
      
      for(int i=0; i<2; i++)  
      { rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }
      
      rfoot_trajectory_euler_support_(0) = 0;
      rfoot_trajectory_euler_support_(1) = 0;
      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj,t_start_ + t_rest_init_ + t_double1_,t_start_ + t_total_ - t_rest_last_ - t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0)
    {
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_euler_support_.setZero();

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
 
      if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + t_rest_temp, t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0); }
      else
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
         
      for(int i=0; i<2; i++)
      { lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_ + t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }

      lfoot_trajectory_euler_support_(0) = 0;
      lfoot_trajectory_euler_support_(1) = 0;  
      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
  }
  else
  {
    if(foot_step_(current_step_num_,6) == 1)
    {
      lfoot_trajectory_euler_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if (foot_step_(current_step_num_,6) == 0)
    {
      rfoot_trajectory_euler_support_.setZero();
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

      for(int i=0; i<3; i++)
      {
        lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
  }  
}
 
void CustomController::preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C)
{
    A.resize(3,3);
    A(0,0) = 1.0;
    A(0,1) = dt;
    A(0,2) = dt*dt*0.5;
    A(1,0) = 0;
    A(1,1) = 1.0;
    A(1,2) = dt;
    A(2,0) = 0;
    A(2,1) = 0;
    A(2,2) = 1;
    
    B.resize(3);
    B(0) = dt*dt*dt/6;
    B(1) = dt*dt/2;
    B(2) = dt;
    
    C.resize(1,3);
    C(0,0) = 1;
    C(0,1) = 0;
    C(0,2) = -0.71/9.81;

    Eigen::MatrixXd A_bar;
    Eigen::VectorXd B_bar;

    B_bar.resize(4);    
    B_bar.segment(0,1) = C*B;
    B_bar.segment(1,3) = B;
    
    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();
    
    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(4,4);
    I_bar.resize(4,1);
    F_bar.resize(4,3);
    F_bar.setZero();

    F_bar.block<1,3>(0,0) = C*A;
    F_bar.block<3,3>(1,0) = A;
    
    I_bar.setZero();
    I_bar(0,0) = 1.0;

    A_bar.block<4,1>(0,0) = I_bar;
    A_bar.block<4,3>(0,1) = F_bar;
    
    Eigen::MatrixXd Qe;
    Qe.resize(1,1);
    Qe(0,0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1,1);
    R(0,0) = 0.000001;

    Eigen::MatrixXd Qx;
    Qx.resize(3,3);
    Qx.setZero();

    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3,3);
    Q_bar.setZero();
    Q_bar(0,0) = Qe(0,0);

    Eigen::Matrix4d K;
    
    K(0,0) = 1083.572780788710;
    K(0,1) = 586523.188429418020;  
    K(0,2) = 157943.283121116518;
    K(0,3) = 41.206077691894;
    K(1,0) = 586523.188429418020;
    K(1,1) = 319653984.254277825356;
    K(1,2) = 86082274.531361579895;
    K(1,3) = 23397.754069026785;
    K(2,0) = 157943.283121116518;
    K(2,1) = 86082274.531361579895;
    K(2,2) = 23181823.112113621086;
    K(2,3) = 6304.466397614751;
    K(3,0) = 41.206077691894;
    K(3,1) = 23397.754069026785;
    K(3,2) = 6304.466397614751;
    K(3,3) = 2.659250532188;
    
    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1,1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1,1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(4,4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();
    
    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;
    
    Eigen::MatrixXd Ac_bar_tran(4,4);
    Ac_bar_tran = Ac_bar.transpose();
    
    Gi.resize(1,1); Gx.resize(1,3);
    Gi(0,0) = 872.3477 ; //Temp_mat_inv * B_bar_tran * K * I_bar ;
    //Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;  
    Gx(0,0) = 945252.1760702;
    Gx(0,1) = 256298.6905049;
    Gx(0,2) = 542.0544196;
    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.resize(4, NL);
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = - Ac_bar_tran * K * I_bar;

    for(int i = 0; i < NL; i++)
    {
        X_bar.block<4,1>(0,i) = X_bar_col;
        X_bar_col = Ac_bar_tran*X_bar_col;
    }           

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0,0);
    
    for(int i = 0; i < NL; i++)
    {
        Gd.segment(i,1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i) ;
    }
    
}

void CustomController::previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY,
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{
    
    int zmp_size;
    zmp_size = ref_zmp_.col(1).size();
    Eigen::VectorXd px_ref, py_ref;
    px_ref.resize(zmp_size);
    py_ref.resize(zmp_size);
    
    for(int i = 0; i < zmp_size; i++)
    {
        px_ref(i) = ref_zmp_(i,0);
        py_ref(i) = ref_zmp_(i,1);
    }
        
    Eigen::VectorXd px, py;
    px.resize(1); py.resize(1);
    
    if(tick == 0 && current_step_num_ == 0)
    {
        preview_x_b.setZero(); preview_y_b.setZero();
        preview_x.setZero(); preview_y.setZero();
        preview_x_b(0) = x_i;  
        preview_y_b(0) = y_i;   
        preview_x(0) = x_i;
        preview_y(0) = y_i;
        UX = 0; UY = 0;
        cout << "preview X state : " << preview_x(0) << "," << preview_x(1) << "," << preview_x(2) << endl;
        cout << "preview Y state : " << preview_y(0) << "," << preview_y(1) << "," << preview_y(2) << endl;
    }
    else
    {     
        preview_x = xs; preview_y = ys;
            
        preview_x_b(0) = preview_x(0) - preview_x(1)*0.0005;  
        preview_y_b(0) = preview_y(0) - preview_y(1)*0.0005;
        preview_x_b(1) = preview_x(1) - preview_x(2)*0.0005;
        preview_y_b(1) = preview_y(1) - preview_y(2)*0.0005;
        preview_x_b(2) = preview_x(2) - UX*0.0005;
        preview_y_b(2) = preview_y(2) - UY*0.0005;
        
    }      
    px = C*preview_x;
    py = C*preview_y;
    
    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

    for(int i = 0; i < NL; i++)
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(tick + 1 + i) - px_ref(tick + i));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(tick + 1 + i) - py_ref(tick + i));
    }
    
    Eigen::MatrixXd del_ux(1,1);
    Eigen::MatrixXd del_uy(1,1);
    del_ux.setZero();
    del_uy.setZero();
    
    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x - preview_x_b);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y - preview_y_b);

    if(walking_tick_mj == 0)
    {
      del_zmp.setZero(); cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }
    
    del_ux(0,0) = -(px(0) - px_ref(tick))*Gi(0,0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0,0) = -(py(0) - py_ref(tick))*Gi(0,0) - GX_Y(0) - sum_Gd_py_ref;
    
    UX = UX + del_ux(0,0);
    UY = UY + del_uy(0,0);

    XD = A*preview_x + B*UX;
    YD = A*preview_y + B*UY;
    //SC_err_compen(XD(0), YD(0));
    if(walking_tick_mj == 0)
    {      
      zmp_err_(0) = 0;
      zmp_err_(1) = 0;
    }
    else
    {
      zmp_err_(0) = zmp_err_(0) + (px_ref(tick) - zmp_measured_LPF_(0))*0.0005;
      zmp_err_(1) = zmp_err_(1) + (py_ref(tick) - zmp_measured_LPF_(1))*0.0005;
    }    

    cp_desired_(0) = XD(0) + XD(1)/wn;
    cp_desired_(1) = YD(0) + YD(1)/wn;

    SC_err_compen(com_support_current_(0), com_support_current_(1));

    cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0)/wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1)/wn;      

    del_zmp(0) = 1.01*(cp_measured_(0) - cp_desired_(0));
    del_zmp(1) = 1.01*(cp_measured_(1) - cp_desired_(1));

    CLIPM_ZMP_compen_MJ(del_zmp(0), del_zmp(1));
        
}

// void CustomController::SC_err_compen(double x_des, double y_des)
// { 
//   if(walking_tick_mj == 0)
//   {
//     SC_com.setZero();
//   }
//   if (walking_tick_mj == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1) // step change 1 tick 이전
//   {  
//     sc_err_before.setZero();
//     sc_err_before(0) = x_des - com_support_current_(0); // 1.3으로 할꺼면 마지막에 더해야됨. SC_com을 이 함수보다 나중에 더하기 때문에
//     sc_err_before(1) = y_des - com_support_current_(1);
//   }

//   if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change 
//   { 
//     sc_err_after.setZero();
//     sc_err_after(0) = x_des - com_support_current_(0); 
//     sc_err_after(1) = y_des - com_support_current_(1);
//     sc_err = sc_err_after - sc_err_before;
//   } 

//   if(current_step_num_ != 0)
//   { 
//     SC_com(0) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + t_total_, sc_err(0), 0, 0.0, 0.0);
//     SC_com(1) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + t_total_, sc_err(1), 0, 0.0, 0.0);
//   }

//   if(current_step_num_ != total_step_num_ - 1)
//   {
//     if(current_step_num_ != 0 && walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_total_)
//     {
//     com_support_current_(0) = com_support_current_(0) + SC_com(0);
//     com_support_current_(1) = com_support_current_(1) + SC_com(1);
//     }
//   }
//   else if(current_step_num_ == total_step_num_ - 1)
//   {
//     if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 2*t_total_)
//     {  
//     com_support_current_(0) = com_support_current_(0) + SC_com(0);
//     com_support_current_(1) = com_support_current_(1) + SC_com(1);
//     }
//   }
// }

void CustomController::SC_err_compen(double x_des, double y_des)
{ 
  if(walking_tick_mj == 0)
  {
    SC_com.setZero();
  }
  if (walking_tick_mj == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1) // step change 1 tick 이전
  {  
    sc_err_before.setZero();
    sc_err_before(0) = com_support_current_(0) - foot_step_support_frame_(current_step_num_,0) ; // 1.3으로 할꺼면 마지막에 더해야됨. SC_com을 이 함수보다 나중에 더하기 때문에
    // sc_err_before(1) = y_des - com_support_current_(1);
  }

  if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change 
  { 
    sc_err_after.setZero();
    sc_err_after(0) = com_support_current_(0) ; 
    // sc_err_after(1) = y_des - com_support_current_(1);
    sc_err = sc_err_after - sc_err_before;
  } 

  if(current_step_num_ != 0)
  { 
    SC_com(0) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05*hz_, sc_err(0), 0, 0.0, 0.0);
    SC_com(1) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05*hz_, sc_err(1), 0, 0.0, 0.0);
  }

  if(current_step_num_ != total_step_num_ - 1)
  {
    if(current_step_num_ != 0 && walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_total_)
    {
      com_support_cp_(0) = com_support_current_(0) - SC_com(0);
    }
    else
    {
      com_support_cp_(0) = com_support_current_(0); 
    }
  }  
  else if(current_step_num_ == total_step_num_ - 1)
  {
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 2*t_total_)
    { com_support_cp_(0) = com_support_current_(0) - SC_com(0); }
  }
}

void CustomController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);
  
  pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7*(com_desired_(0) - 0.15*damping_x - com_support_current_(0));//- 0.01 * zmp_err_(0) * 0;
  pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7*(com_desired_(1) - 0.6*damping_y - com_support_current_(1)) ;//- 0.01 * zmp_err_(1) * 0;
  pelv_trajectory_support_.translation()(2) = com_desired_(2);
  // MJ_graph << com_desired_(0) << "," << com_support_current_(0) << "," << com_desired_(1) << "," << com_support_current_(1) << endl;
  Eigen::Vector3d Trunk_trajectory_euler;
  Trunk_trajectory_euler.setZero();

  if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
  { Trunk_trajectory_euler(2) = pelv_support_euler_init_(2); }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
  { Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0); }
  else
  { Trunk_trajectory_euler(2) = z_rot/2.0; } 

  // P_angle_i = P_angle_i + (0 - P_angle)*del_t;
  // Trunk_trajectory_euler(1) = 0.05*(0.0 - P_angle) + 1.5*P_angle_i;
  if(walking_tick_mj == 0)
  { P_angle_input = 0; R_angle_input = 0; }
  
  P_angle_input_dot = 1.5*(0.0 - P_angle) - 0.01*P_angle_input;
  R_angle_input_dot = 1.0*(0.0 - R_angle) - 0.005*R_angle_input;
  
  P_angle_input = P_angle_input + P_angle_input_dot*del_t;
  R_angle_input = R_angle_input + R_angle_input_dot*del_t;
  
  if(R_angle_input > 0.0262)
  { R_angle_input = 0.0262; }
  else if(R_angle_input < -0.0262)
  { R_angle_input = -0.0262; }

  if(P_angle_input > 0.0262)
  { P_angle_input = 0.0262; }
  else if(P_angle_input < -0.0262)
  { P_angle_input = -0.0262; }

  Trunk_trajectory_euler(0) = R_angle_input;
  Trunk_trajectory_euler(1) = P_angle_input;

  // MJ_graph << R_angle * 180 / 3.141592 << "," << Trunk_trajectory_euler(0) << "," << P_angle * 180 / 3.141592 << "," << Trunk_trajectory_euler(1) << endl;
    
  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
     
}

void CustomController::supportToFloatPattern()
{
  //lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
  //rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
  pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*pelv_trajectory_support_;
  lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lfoot_trajectory_support_;
  rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rfoot_trajectory_support_;
}

void CustomController::getComTrajectory()
{
  if(walking_tick_mj == 0)  
  {
    Gi_.setZero(); Gx_.setZero(); Gd_.setZero();
    preview_Parameter(1.0/hz_, 16*hz_/10, Gi_, Gd_, Gx_, A_, B_, C_);
    xs_(0) = xi_; xs_(1) = 0; xs_(2) = 0;
    ys_(0) = yi_; ys_(1) = 0; xs_(2) = 0;
    UX_ = 0; UY_ = 0;
    xd_ = xs_;
  }

  if(current_step_num_ == 0)
  { zmp_start_time_ = 0.0; }
  else
  { zmp_start_time_ = t_start_; }
       
  previewcontroller(0.0005, 3200, walking_tick_mj - zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, C_, xd_, yd_);
   
  xs_ = xd_; ys_ = yd_;

  com_desired_(0) = xd_(0);
  com_desired_(1) = yd_(0);
  com_desired_(2) = pelv_support_start_.translation()(2);

  //SC_err_compen(com_desired_(0), com_desired_(1)); 

  if (walking_tick_mj == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1)  
  {
    Eigen::Vector3d com_pos_prev;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel_prev;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d com_acc_prev;
    Eigen::Vector3d com_acc;
    Eigen::Matrix3d temp_rot;
    Eigen::Vector3d temp_pos;
    
    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5));
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);     
    
    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);
     
    com_vel_prev(0) = xs_(1);
    com_vel_prev(1) = ys_(1);
    com_vel_prev(2) = 0.0;
    com_vel = temp_rot*com_vel_prev;

    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

    xs_(0) = com_pos(0);
    ys_(0) = com_pos(1);
    xs_(1) = com_vel(0);
    ys_(1) = com_vel(1);
    xs_(2) = com_acc(0);
    ys_(2) = com_acc(1);
  }
 
}

void CustomController::computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des)
{
    Eigen::Vector3d R_r, R_D, L_r, L_D ;

    L_D << 0.11 , +0.1025, -0.1025;
    R_D << 0.11 , -0.1025, -0.1025;

    L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*L_D - float_lleg_transform.translation());
    R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*R_D - float_rleg_transform.translation());

    double R_C = 0, L_C = 0, L_upper = 0.351, L_lower = 0.351 , R_alpha = 0, L_alpha = 0;

    L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
    R_C = sqrt( pow(R_r(0),2) + pow(R_r(1),2) + pow(R_r(2),2) );

    q_des(3) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
    q_des(9) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
    L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
    R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));

    q_des(4)  = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2))) - L_alpha ;
    q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3)-q_des(4));
    L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9)-q_des(10));
    R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

    L_Hip_rot_mat.setZero(); R_Hip_rot_mat.setZero();

    L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

    q_des(0) =  atan2(-L_Hip_rot_mat(0,1),L_Hip_rot_mat(1,1)); // Hip yaw
    q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
    q_des(2) =  atan2(-L_Hip_rot_mat(2,0), L_Hip_rot_mat(2,2)) ; // Hip pitch
    q_des(3) =  q_des(3) ; // Knee pitch
    q_des(4) =  q_des(4) ; // Ankle pitch
    q_des(5) =  atan2( L_r(1), L_r(2) ); // Ankle roll

    q_des(6) =  atan2(-R_Hip_rot_mat(0,1),R_Hip_rot_mat(1,1));
    q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2,0), R_Hip_rot_mat(2,2));
    q_des(9) = q_des(9) ;
    q_des(10) = q_des(10) ;
    q_des(11) =  atan2( R_r(1), R_r(2) );

    if(walking_tick_mj == 0)
    { sc_joint_err.setZero(); }

    if (walking_tick_mj == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1) // step change 1 tick 이전
    { //5.3, 0
      sc_joint_before.setZero();
      sc_joint_before = q_des;     
    }
    if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change 
    { //5.3005, 1
      sc_joint_after.setZero();
      sc_joint_after = q_des; 

      sc_joint_err = sc_joint_after - sc_joint_before;
    }
    if(current_step_num_ != 0)
    {
      for(int i = 0; i < 12 ; i ++)
      { SC_joint(i) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.005*hz_, sc_joint_err(i), 0.0, 0.0, 0.0); }
      
      if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 0.005*hz_)
      {
        q_des = q_des - SC_joint;    
      }
    }

}

void CustomController::GravityCalculate_MJ()
{
  double grav_gain = 0.0;
  double contact_gain = 0.0;
  Eigen::Vector12d A; double B = 0.0;
   
  if(walking_tick_mj < t_start_ + t_rest_init_ )
  {
    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);    
    Gravity_SSP_.setZero();
    contact_gain = 1.0;
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, contact_gain, 1); }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, contact_gain, 0); }  
  }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ ) // 0.03 s  
  {
    contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_, t_start_ + t_rest_init_ + t_double1_, 1.0, 0.0, 0.0, 0.0);
    
    wbc_.set_contact(rd_, 1, 1);       
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
    Gravity_SSP_.setZero();
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, contact_gain, 1); }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, contact_gain, 0); }
  }

  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
  {
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    {
      wbc_.set_contact(rd_, 1, 0);       
      Gravity_SSP_ = wbc_.gravity_compensation_torque(rd_);
      Gravity_SSP_(1) = 1.4*Gravity_SSP_(1);
      Gravity_SSP_(5) = 1.15*Gravity_SSP_(5);
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      wbc_.set_contact(rd_, 0, 1);       
      Gravity_SSP_ = wbc_.gravity_compensation_torque(rd_); 
      Gravity_SSP_(7) = 1.4*Gravity_SSP_(7);
      Gravity_SSP_(11) = 1.15*Gravity_SSP_(11);
    }
    Gravity_DSP_.setZero();
    contact_torque_MJ.setZero();
  }

  else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
  {
    contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ , t_start_ + t_total_ - t_rest_last_ , 0.0, 1.0, 0.0, 0.0);
    Gravity_SSP_.setZero();
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    {   
      wbc_.set_contact(rd_, 1, 1);
      Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);  
      contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, contact_gain, 1);           
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      wbc_.set_contact(rd_, 1, 1);
      Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
      contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, contact_gain, 0);
    }    
  }
  else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
  {
    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
    
    Gravity_SSP_.setZero();
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, 1.0, 1); }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    { contact_torque_MJ = wbc_.contact_force_redistribution_torque_walking(rd_, Gravity_DSP_, A, B, 1.0, 0); }
  }
  
  Gravity_MJ_ = Gravity_DSP_ + Gravity_SSP_ + contact_torque_MJ;
}


void CustomController::parameterSetting()
{
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.08;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    // 1.4 Hz 실험
    t_rest_init_ = 0.27*hz_;
    t_rest_last_ = 0.27*hz_;  
    t_double1_ = 0.03*hz_;
    t_double2_ = 0.03*hz_;
    t_total_= 1.3*hz_;

    // t_rest_init_ = 0.23*hz_;
    // t_rest_last_ = 0.23*hz_;  
    // t_double1_ = 0.02*hz_;
    // t_double2_ = 0.02*hz_;
    // t_total_= 1.2*hz_;

    t_temp_ = 4.0*hz_;
    t_last_ = t_total_ + t_temp_ ;
    t_start_ = t_temp_ + 1 ;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
    foot_height_ = 0.055; // 실험 제자리 0.04 , 전진 0.05 시뮬 0.04
}

void CustomController::updateNextStepTime()
{
    if(walking_tick_mj == t_last_)
    {
        if(current_step_num_ != total_step_num_-1)
        {
          t_start_ = t_last_ + 1 ;
          t_start_real_ = t_start_ + t_rest_init_;
          t_last_ = t_start_ + t_total_ -1;
          current_step_num_ ++;
        }
    }
   if(current_step_num_ == total_step_num_-1 && walking_tick_mj >= t_last_ + t_total_)
   {
     walking_enable_ = false; cout << "Last " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_(2)*180/3.141592 << endl;
   }
   walking_tick_mj ++;
}

void CustomController::CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref)
{ 
  double Kp_x_ssp, Kv_x_ssp;
  double Kp_y_ssp, Kv_y_ssp;
  Kp_x_ssp = 30; Kv_x_ssp = 2;
  Kp_y_ssp = 40; Kv_y_ssp = 2; 
  double del_t = 0.0005;

  if(walking_tick_mj == 0)
  {
    A_x_ssp.resize(2,2);
    B_x_ssp.resize(2,1);
    Ad_x_ssp.resize(2,2);
    Bd_x_ssp.resize(2,1);
    C_x_ssp.resize(1,2);
    D_x_ssp.resize(1,1); 
    
    A_y_ssp.resize(2,2);
    B_y_ssp.resize(2,1);
    Ad_y_ssp.resize(2,2);
    Bd_y_ssp.resize(2,1);
    C_y_ssp.resize(1,2);
    D_y_ssp.resize(1,1);

    ff_gain_x_ssp.resize(1,1);
    ff_gain_y_ssp.resize(1,1); 

    K_x_ssp.resize(1,2);
    K_y_ssp.resize(1,2); 

    X_x_ssp.setZero(); 
    Y_x_ssp.resize(1,1);

    X_y_ssp.setZero(); 
    Y_y_ssp.resize(1,1); 

    K_x_ssp(0,0) = 0.0083;      
    K_x_ssp(0,1) = 0.19; 
    // Control pole : -5 , damping : 0.7 (실제 로봇) // Control pole : -7 , damping : 0.9 (시뮬레이션)
    K_y_ssp(0,0) = -0.375;  
    K_y_ssp(0,1) = 0.125; 
    
    // Define the state space equation 
    A_x_ssp(0,0) = 0;
    A_x_ssp(0,1) = 1;
    A_x_ssp(1,0) = - Kp_x_ssp;
    A_x_ssp(1,1) = - Kv_x_ssp;

    B_x_ssp(0,0) = 0;
    B_x_ssp(1,0) = Kp_x_ssp;

    Ad_x_ssp(0,0) = 1 - 0.5 * Kp_x_ssp * del_t * del_t;
    Ad_x_ssp(0,1) = del_t - 0.5 * Kv_x_ssp * del_t * del_t;
    Ad_x_ssp(1,0) = - Kp_x_ssp * del_t;
    Ad_x_ssp(1,1) = 1 - Kv_x_ssp * del_t;

    Bd_x_ssp(0,0) = 0.5*Kp_x_ssp*del_t*del_t;
    Bd_x_ssp(1,0) = Kp_x_ssp*del_t;

    C_x_ssp(0,0) = 1 + zc_/GRAVITY * Kp_x_ssp;
    C_x_ssp(0,1) = zc_/GRAVITY * Kv_x_ssp;

    D_x_ssp(0,0) = -zc_/GRAVITY * Kp_x_ssp;
 
    ff_gain_x_ssp = (-(C_x_ssp - D_x_ssp*K_x_ssp)*((A_x_ssp - B_x_ssp*K_x_ssp).inverse())*B_x_ssp + D_x_ssp).inverse(); 

    A_y_ssp(0,0) = 0;
    A_y_ssp(0,1) = 1;
    A_y_ssp(1,0) = - Kp_y_ssp;
    A_y_ssp(1,1) = - Kv_y_ssp;

    B_y_ssp(0,0) = 0;
    B_y_ssp(1,0) = Kp_y_ssp;

    Ad_y_ssp(0,0) = 1 - 0.5 * Kp_y_ssp * del_t * del_t;
    Ad_y_ssp(0,1) = del_t - 0.5 * Kv_y_ssp * del_t * del_t;
    Ad_y_ssp(1,0) = - Kp_y_ssp * del_t;
    Ad_y_ssp(1,1) = 1 - Kv_y_ssp * del_t;

    Bd_y_ssp(0,0) = 0.5*Kp_y_ssp*del_t*del_t;
    Bd_y_ssp(1,0) = Kp_y_ssp*del_t;

    C_y_ssp(0,0) = 1 + zc_/GRAVITY * Kp_y_ssp;
    C_y_ssp(0,1) = zc_/GRAVITY * Kv_y_ssp;

    D_y_ssp(0,0) = -zc_/GRAVITY * Kp_y_ssp;
 
    ff_gain_y_ssp = (-(C_y_ssp - D_y_ssp*K_y_ssp)*((A_y_ssp - B_y_ssp*K_y_ssp).inverse())*B_y_ssp + D_y_ssp).inverse();     
  }  

  //X_x_ssp(0) = com_float_current_(0);
  X_x_ssp(0) = com_support_current_(0); 

  if(foot_step_(current_step_num_, 6) == 1) // 왼발 지지
  { X_y_ssp(0) = com_support_current_(1) - rfoot_support_current_.translation()(1)*0.5; } 
  else if(foot_step_(current_step_num_, 6) == 0)
  { X_y_ssp(0) = com_support_current_(1) - lfoot_support_current_.translation()(1)*0.5; } 
  
  U_ZMP_x_ssp = - (K_x_ssp(0,0)*X_x_ssp(0) + K_x_ssp(0,1)*preview_x(1)) + XZMP_ref * ff_gain_x_ssp(0,0);
  U_ZMP_y_ssp = - (K_y_ssp(0,0)*X_y_ssp(0) + K_y_ssp(0,1)*preview_y(1)) + YZMP_ref * ff_gain_y_ssp(0,0); 
  
  U_ZMP_x_ssp_LPF =  1/(1+2*M_PI*3.0*del_t)*U_ZMP_x_ssp_LPF + (2*M_PI*3.0*del_t)/(1+2*M_PI*3.0*del_t)*U_ZMP_x_ssp;
  U_ZMP_y_ssp_LPF =  1/(1+2*M_PI*6.0*del_t)*U_ZMP_y_ssp_LPF + (2*M_PI*6.0*del_t)/(1+2*M_PI*6.0*del_t)*U_ZMP_y_ssp;       
  if(walking_tick_mj == 0)
  { U_ZMP_x_ssp_LPF = U_ZMP_x_ssp; U_ZMP_y_ssp_LPF = U_ZMP_y_ssp; }

  damping_x = U_ZMP_x_ssp_LPF ;
  damping_y = U_ZMP_y_ssp_LPF ;

  if(damping_x > 0.02)  
  { damping_x = 0.02; }
  else if(damping_x < - 0.02)
  { damping_x = -0.02; } 

  if(damping_y > 0.03) // 로봇 0.03, 시뮬 0.02
  { damping_y = 0.03; }
  else if(damping_y < - 0.03)
  { damping_y = -0.03; }
  
  // MJ_graph << cp_desired_(1) << "," << cp_measured_(1) << "," << com_float_current_(1) << "," << X_y_ssp(0) << "," << damping_y << endl;  
}

void CustomController::hip_compensator()
{  
  double left_hip_roll = -0.3*DEG2RAD, right_hip_roll = -0.3*DEG2RAD, left_hip_roll_first = -0.50*DEG2RAD, right_hip_roll_first = -0.50*DEG2RAD, //실험, 제자리 0.6, 0.4
  left_hip_pitch = 0.4*DEG2RAD, right_hip_pitch = 0.4*DEG2RAD, left_hip_pitch_first = 0.40*DEG2RAD, right_hip_pitch_first = 0.40*DEG2RAD, // 실험 , 제자리 0.75deg
  left_ank_pitch = 0.0*DEG2RAD, right_ank_pitch = 0.0*DEG2RAD, left_ank_pitch_first = 0.0*DEG2RAD, right_ank_pitch_first = 0.0*DEG2RAD,
      left_hip_roll_temp = 0.0, right_hip_roll_temp = 0.0, left_hip_pitch_temp = 0.0, right_hip_pitch_temp = 0.0, left_ank_pitch_temp = 0.0, right_ank_pitch_temp = 0.0, temp_time = 0.05*hz_;


  if (current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_hip_roll_first, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_hip_pitch_first, 0.0, 0.0);
        left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_ank_pitch_first, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_roll_first, 0.0, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_pitch_first, 0.0, 0.0, 0.0);
        left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_ank_pitch_first, 0.0, 0.0, 0.0);
      }      
      else
      { left_hip_roll_temp = 0.0; left_hip_pitch_temp = 0.0; left_ank_pitch_temp = 0.0; }      
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_hip_roll_first, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_hip_pitch_first, 0.0, 0.0);
        right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_ank_pitch_first, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_roll_first, 0.0, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_pitch_first, 0.0, 0.0, 0.0);
        right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_ank_pitch_first, 0.0, 0.0, 0.0);
      }        
      else
      { right_hip_roll_temp = 0.0; right_hip_pitch_temp = 0.0; right_ank_pitch_temp = 0.0; }
    }    
  }
  else
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_roll, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_pitch, 0.0, 0.0);
        left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_ank_pitch, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_roll, 0.0, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_pitch, 0.0, 0.0, 0.0);
        left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_ank_pitch, 0.0, 0.0, 0.0);
      }      
      else
      { left_hip_roll_temp = 0; left_hip_pitch_temp = 0; left_ank_pitch_temp = 0; }      
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_roll, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_pitch, 0.0, 0.0);
        right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_ank_pitch, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_roll, 0.0, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_pitch, 0.0, 0.0, 0.0);
        right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_ank_pitch, 0.0, 0.0, 0.0);
      }        
      else
      { right_hip_roll_temp = 0; right_hip_pitch_temp = 0; right_ank_pitch_temp = 0.0; }
    }    
  }  
 
  ref_q_(1) = ref_q_(1) - left_hip_roll_temp;
  ref_q_(7) = ref_q_(7) + right_hip_roll_temp;
  ref_q_(2) = ref_q_(2) - left_hip_pitch_temp;
  ref_q_(8) = ref_q_(8) - right_hip_pitch_temp;
  ref_q_(4) = ref_q_(4) - left_ank_pitch_temp;
  ref_q_(10) = ref_q_(10) - right_ank_pitch_temp;
}

void CustomController::Compliant_control(Eigen::Vector12d desired_leg_q)
{     
  Eigen::Vector12d current_u;
  double del_t = 0.0, Kp = 0.0;
  del_t = 1/hz_; Kp = 25.0; // 실험
  //Kp = 20.0; // 시뮬

  if(walking_tick_mj == 0)  
  {
    for(int i = 0; i < 12; i++)
    { DOB_IK_output_b_(i) = rd_.q_(i); DOB_IK_output_(i) = rd_.q_(i); current_u(i) = rd_.q_(i); }    
  }
 
  if(walking_tick_mj > 0)
  {
    for (int i = 0; i < 12; i++)
    {
      current_u(i) = (rd_.q_(i) - (1 - Kp*del_t)*q_prev_MJ_(i)) / (Kp*del_t);
    }
  }
 
  Eigen::Vector12d d_hat;    
  d_hat = current_u - DOB_IK_output_b_ ;
 
  if(walking_tick_mj == 0)
    d_hat_b = d_hat;

  d_hat = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*d_hat + 1/(1+2*M_PI*8.0*del_t)*d_hat_b;

  double default_gain = 0.0;
  double compliant_gain = 0.0;
  double compliant_tick = 0.1*hz_;
  double gain_temp = 0.0;
  for (int i = 0; i < 12; i ++)
  {
    if(i < 6)
    {
      gain_temp = default_gain;
      
      if (foot_step_(current_step_num_,6) == 0)
      {
        if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
        {
          gain_temp = default_gain;
        }
        else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_)
        {
          gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
        }
        else
        {
          gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
        }
      }
      else
      {
        gain_temp = default_gain;
      }
             
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);  
    }
    else
    {      
      gain_temp = default_gain;
      
      if (foot_step_(current_step_num_,6) == 1) // 왼발 지지 상태
      {
        if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
        {
          gain_temp = default_gain;
        }
        else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_)
        {
          gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
        }
        else
        {
          gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
        }
      }
      else // 오른발 지지 상태
      {
        gain_temp = default_gain;
      }
      
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);  
    }
  }   

  d_hat_b = d_hat;
  DOB_IK_output_b_ = DOB_IK_output_;
}

void CustomController::CP_compen_MJ()
{
  double alpha = 0;
  double F_R = 0, F_L = 0;

  // Tau_R.setZero(); Tau_L.setZero(); 

  Tau_CP.setZero();

  alpha = (com_float_current_(1) - rfoot_float_current_.translation()(1))/(lfoot_float_current_.translation()(1) - rfoot_float_current_.translation()(1));
  
  if(alpha > 1)
  { alpha = 1; }
  else if(alpha < 0)
  { alpha = 0; } 

  F_R = (1 - alpha) * rd_.com_.mass * GRAVITY;
  F_L = alpha * rd_.com_.mass * GRAVITY; 

  Tau_CP(4) = F_L * del_zmp(0); // L pitch
  Tau_CP(10) = F_R * del_zmp(0); // R pitch

  Tau_CP(5) = -F_L * del_zmp(1); // L roll
  Tau_CP(11) = -F_R * del_zmp(1); // R roll
}
void CustomController::updateInitialStateJoy()
{
  if(walking_tick_mj == 0)
  {
    calculateFootStepTotal_MJoy(); // joystick&pedal Footstep
    joy_enable_ = true;
    std::cout << "step_length : " << joystick_input_(0) << " trigger(z) : " << joystick_input_(1) << " theta : " << joystick_input_(2)<< std::endl;
        
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].Rotm); //ZYX multiply
    
    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));   
    
    pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].Rotm;

    pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
    //pelv_float_init_.translation()(0) += 0.11;
    
    pelv_float_init_.translation()(0) = 0;
    pelv_float_init_.translation()(1) = 0;

    lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].Rotm;
    lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
    
    lfoot_float_init_.translation()(0) = 0;
    lfoot_float_init_.translation()(1) = 0.1225;

    rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].Rotm;
    rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
    
    rfoot_float_init_.translation()(0) = 0;
    rfoot_float_init_.translation()(1) = -0.1225;

    com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
    
    com_float_init_(0) = 0;
    com_float_init_(1) = 0;

    if(aa == 0)
    {
      lfoot_float_init_.translation()(1) = 0.1025;
      rfoot_float_init_.translation()(1) = -0.1025;
      // t_temp_ = 4.0*hz_
      aa = 1;
    }
    cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_(2)*180/3.141592 << endl;
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(0, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(0, 6) == 1)
    { ref_frame = lfoot_float_init_; }
         
    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
    com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
     
    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();
     
    if(foot_step_(0,6) == 1) //left suppport foot
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    else
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }

    pelv_support_start_ = pelv_support_init_;
    total_step_num_ = foot_step_.col(1).size();
    
    xi_ = com_support_init_(0); // preview parameter
    yi_ = com_support_init_(1);
    zc_ = com_support_init_(2);
    
  }
  else if(current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
  { 
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].Rotm); //ZYX multiply

    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    
    pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].Rotm;

    pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Pelvis].xpos);
    //pelv_float_init_.translation()(0) += 0.11;
      
    lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].Rotm;
    lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Left_Foot].xpos);  // 지면에서 Ankle frame 위치
    
    rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].Rotm;
    rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
    
    com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_),rd_.link_[COM_id].xpos); // 지면에서 CoM 위치   
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { ref_frame = lfoot_float_init_; }     

    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
    com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
      
  }
  if(walking_tick_mj == t_start_) 
  {
    if(joy_input_enable_ == true){
      joystick_input_(0) = (joystick_input(0) + 1)/2 ; //FW
      joystick_input_(3) = (joystick_input(3) + 1)/2 ; //BW
      // joystick_input_(1) = joystick_input(1);
      joystick_input_(2) = - joystick_input(2);
      joystick_input_(1) = joystick_input_(0)+ abs(joystick_input_(2)) + joystick_input_(3);
    }

    if(joystick_input_(1) > 0){
      calculateFootStepTotal_MJoy();
      total_step_num_ = foot_step_.col(1).size();
      joy_enable_ = true;
      std::cout << "step_length : " << joystick_input_(0) << " trigger(z) : " << joystick_input_(1) << " theta : " << joystick_input_(2)<< std::endl;
    }
    else if(joy_enable_ == true){
      calculateFootStepTotal_MJoy_End();
      total_step_num_ = foot_step_.col(1).size();
      joy_enable_ = false;
      joy_input_enable_ = false;
      joystick_input_(1) = -1.0;
    }
  }
}

void CustomController::calculateFootStepTotal_MJoy()
{
  double width = 0.1225;
  double length  = 0.09;
  double lengthb = 0.07;
  double theta = 10 * DEG2RAD;
  double width_buffer = 0.0;
  double temp;
  int temp2;
  int index = 3;

  double length_total = joystick_input_(0) * length - joystick_input_(3) * lengthb;

  joy_index_ ++;
  foot_step_.resize(joy_index_ + index, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(joy_index_ + index, 7);
  foot_step_support_frame_.setZero();

  if(walking_tick_mj != 0){
    // for(int i=0; i<joy_index_ + 2 ; i++){
    for(int i=0; i<foot_step_joy_temp_.col(1).size(); i++){
      foot_step_(i,0) = foot_step_joy_temp_(i,0);
      foot_step_(i,1) = foot_step_joy_temp_(i,1);
      foot_step_(i,5) = foot_step_joy_temp_(i,5);
      foot_step_(i,6) = foot_step_joy_temp_(i,6);
    }
  }

  foot_step_(1,6) = 0;
  if(aa == 0) 
  {
    width_buffer = 0.01;
    joystick_input_(0) = 0;
    joystick_input_(2) = 0;
    joystick_input_(3) = 0;
  }

  if(joy_index_ < 3)
  {
    temp = 1;
    temp2 = 0;

    foot_step_(0,5) = temp2 * joystick_input_(2) * theta; //0.0;
    foot_step_(0,0) =    (width - width_buffer) * sin(foot_step_(0,5)) + temp2 * length_total * cos(foot_step_(foot_step_(0,5))); //0.0;
    foot_step_(0,1) =  - (width - width_buffer) * cos(foot_step_(0,5)) + temp2 * length_total * sin(foot_step_(foot_step_(0,5)));
    foot_step_(0,6) = 1.0;
    temp2++;

    foot_step_(1,5) = temp2 * joystick_input_(2) * theta; //0.0;
    foot_step_(1,0) = - width * sin(foot_step_(1,5)) + temp2 * length_total * cos(foot_step_(foot_step_(1,5)));
    foot_step_(1,1) =   width * cos(foot_step_(1,5)) + temp2 * length_total * sin(foot_step_(foot_step_(1,5)));
    foot_step_(1,6) = 0.0;
    temp2++;

    foot_step_(2,5) = temp2 * joystick_input_(2) * theta;
    foot_step_(2,0) =   width * sin(foot_step_(2,5)) + temp2 * length_total * cos(foot_step_(foot_step_(2,5)));
    foot_step_(2,1) = - width * cos(foot_step_(2,5)) + temp2 * length_total * sin(foot_step_(foot_step_(2,5)));
    foot_step_(2,6) = 1.0;
    temp2++;

    foot_step_(3,5) = temp2 * joystick_input_(2) * theta;
    foot_step_(3,0) = - width * sin(foot_step_(3,5)) + temp2 * length_total * cos(foot_step_(foot_step_(3,5)));
    foot_step_(3,1) =   width * cos(foot_step_(3,5)) + temp2 * length_total * sin(foot_step_(foot_step_(3,5)));
    foot_step_(3,6) = 0.0;
  }
  else
  {
    if(foot_step_(joy_index_,6) ==1) temp = 1;
    else if(foot_step_(joy_index_,6) ==0) temp = -1;

    for(int i=-1;i<index;i++){
    temp *= -1;

    foot_step_(joy_index_ + i,5) = foot_step_(joy_index_ + i -1 ,5) + joystick_input_(2)*theta;
    foot_step_(joy_index_ + i,0) = foot_step_(joy_index_ + i -1 ,0) + temp * width * (sin(foot_step_(joy_index_ + i -1,5)) + sin(foot_step_(joy_index_ + i,5))) + length_total * cos(foot_step_(joy_index_ + i,5));
    foot_step_(joy_index_ + i,1) = foot_step_(joy_index_ + i -1 ,1) - temp * width * (cos(foot_step_(joy_index_ + i -1,5)) + cos(foot_step_(joy_index_ + i,5))) + length_total * sin(foot_step_(joy_index_ + i,5));
    foot_step_(joy_index_ + i,6) = 0.5 + 0.5*temp;
    }
  }

  foot_step_joy_temp_.resize(joy_index_ + index, 7);
  foot_step_joy_temp_.setZero();
  foot_step_joy_temp_ = foot_step_;

}
void CustomController::calculateFootStepTotal_MJoy_End()
{
  double width = 0.1225;
  double length = 0.10;
  double theta = 10 * DEG2RAD;
  double width_buffer = 0.0;
  double temp;
  int index = 1;

  joy_index_ ++;
  foot_step_.resize(joy_index_ + index, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(joy_index_ + index, 7);
  foot_step_support_frame_.setZero();

  if(walking_tick_mj != 0){    
    for(int i=0; i<joy_index_ + 1 ; i++)
    {
      foot_step_(i,0) = foot_step_joy_temp_(i,0);
      foot_step_(i,1) = foot_step_joy_temp_(i,1);
      foot_step_(i,5) = foot_step_joy_temp_(i,5);
      foot_step_(i,6) = foot_step_joy_temp_(i,6);
    }
  }

  if(foot_step_(joy_index_,6) ==1) temp = 1;
  else if(foot_step_(joy_index_,6) ==0) temp = -1;

  for(int i=-1;i<index;i++){
    temp *= -1;
    foot_step_(joy_index_ + i,5) = foot_step_(joy_index_ + i -1 ,5);
    foot_step_(joy_index_ + i,0) = foot_step_(joy_index_ + i -1 ,0) + temp * 2* width*sin(foot_step_(joy_index_ + i,5));
    foot_step_(joy_index_ + i,1) = foot_step_(joy_index_ + i -1 ,1) - temp * 2* width*cos(foot_step_(joy_index_ + i,5));
    foot_step_(joy_index_ + i,6) = 0.5 + 0.5*temp;
  }

  cout << "-----footstep-position-----" << endl;
  for(int i=0; i<joy_index_ + index ; i++){
    cout << i << " : " << foot_step_(i,6) << " : " << foot_step_(i,5) <<  " : " <<foot_step_(i,0) << " , " << foot_step_(i,1)<<endl;
  }
  cout << "-----footstep-planning-----" << endl;
}
void CustomController::updateNextStepTimeJoy()
{
  if(walking_tick_mj == t_last_)
  {
    if(current_step_num_ != total_step_num_-1)
    {
      t_start_ = t_last_ + 1 ;
      t_start_real_ = t_start_ + t_rest_init_;
      t_last_ = t_start_ + t_total_ -1;
      current_step_num_ ++;
    }
  }

  walking_tick_mj ++;

  if(current_step_num_ == total_step_num_-1 && walking_tick_mj >= t_last_ + t_total_ + 1)
  {
    walking_enable_ = false;
    walking_tick_mj = 0;
    joy_index_ = 0;
    ref_q_ = rd_.q_;
      for(int i = 0; i < 12; i ++)
      {
        Initial_ref_q_(i) = ref_q_(i);
      }
    cout <<  "            end" << endl;
    cout <<  "___________________________" << endl;
    joy_input_enable_ = true;
  }
}

void CustomController::computePlanner()
{
}