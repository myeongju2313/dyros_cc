#include "custom_controller.h"
#include <fstream>

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_)
{
    ControlVal_.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}
void CustomController::taskCommandToCC(TaskCommand tc_)
{
    tc = tc_;
}

ofstream MJ_graph("/home/myeongju/MJ_graph.txt");

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
            initial_flag = 1;
            q_dot_LPF_MJ.setZero();
        } 

        wbc_.set_contact(rd_, 1, 1);  
        Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);
          
        for(int i = 0; i < MODEL_DOF; i++)
        { ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 0.85*Gravity_MJ_(i) ; }        
    }
    else if (tc.mode == 11)
    { 
      if(walking_enable_ == true)
      {
        if(walking_tick_mj == 0)
        {     
            parameterSetting();
            cout << "parameter setting OK" << endl;
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
              ref_q_(i) = DOB_IK_output_(i);
            }            
            hip_compensator();
            GravityCalculate_MJ();
             
            for(int i = 0; i < MODEL_DOF; i++)
            { 
              ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 0.85*Gravity_MJ_(i) ;
            }
        
            desired_q_not_compensated_ = ref_q_;           

            updateNextStepTime();            
        }        
      }
      else
      {
        wbc_.set_contact(rd_, 1, 1);
        Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);
        for(int i = 0; i < MODEL_DOF; i++)
        { ControlVal_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 0.85*Gravity_MJ_(i); }
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
    calculateFootStepTotal();

    com_float_init_ =  rd_.link_[COM_id].xpos ;  

    pelv_float_init_.linear() = rd_.link_[Pelvis].Rotm;
    pelv_float_init_.translation() = rd_.link_[Pelvis].xpos;
    pelv_float_init_.translation()(0) += 0.11;
    

    //lfoot_float_init_.linear().setIdentity();
    lfoot_float_init_.linear() = rd_.link_[Left_Foot].Rotm;
    lfoot_float_init_.translation() = rd_.link_[Left_Foot].xpos; 
    
    //rfoot_float_init_.linear().setIdentity(); 
    rfoot_float_init_.linear() = rd_.link_[Right_Foot].Rotm; 
    rfoot_float_init_.translation() = rd_.link_[Right_Foot].xpos; 

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
    //lfoot_float_init_.linear().setIdentity();
    lfoot_float_init_.linear() = rd_.link_[Left_Foot].Rotm;
    lfoot_float_init_.translation() = rd_.link_[Left_Foot].xpos;      

    //rfoot_float_init_.linear().setIdentity();  
    rfoot_float_init_.linear() = rd_.link_[Right_Foot].Rotm;
    rfoot_float_init_.translation() = rd_.link_[Right_Foot].xpos;
     
    com_float_init_ =  rd_.link_[COM_id].xpos; 
    //pelv_float_init_.linear().setIdentity(); 
    pelv_float_init_.linear() = rd_.link_[Pelvis].Rotm;
    pelv_float_init_.translation() = rd_.link_[Pelvis].xpos;
    pelv_float_init_.translation()(0) += 0.11;

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
    com_float_current_ = rd_.link_[COM_id].xpos; // 지면에서 CoM 위치   

    //lfoot_float_current_.linear().setIdentity();
    lfoot_float_current_.linear() = rd_.link_[Left_Foot].Rotm; 
    lfoot_float_current_.translation() = rd_.link_[Left_Foot].xpos;  // 지면에서 Ankle frame 위치
    
    //rfoot_float_current_.linear().setIdentity();    
    rfoot_float_current_.linear() = rd_.link_[Right_Foot].Rotm;
    rfoot_float_current_.translation() = rd_.link_[Right_Foot].xpos; // 지면에서 Ankle frame
    
    if(foot_step_(current_step_num_, 6) == 0)
    { supportfoot_float_current_ = rfoot_float_current_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { supportfoot_float_current_ = lfoot_float_current_; }
    
    //pelv_float_current_.linear().setIdentity();
    pelv_float_current_.linear() = rd_.link_[Pelvis].Rotm;
    
    pelv_float_current_.translation() = rd_.link_[Pelvis].xpos; 
    pelv_float_current_.translation()(0) += 0.11;
    
    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * pelv_float_current_;   
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * lfoot_float_current_; 
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * rfoot_float_current_; 
    com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_), com_float_current_);
    
    l_ft_ = rd_.ContactForce_FT_raw.segment(0, 6);
    r_ft_ = rd_.ContactForce_FT_raw.segment(6, 6);    

    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_support_current_.translation()(1);

    zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2)); // ZMP X
    zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2)); // ZMP Y
    
    if(walking_tick_mj == 0)
    {
      zmp_measured_LPF_.setZero();
    }
    zmp_measured_LPF_ = (2*M_PI*8.0*del_t)/(1+2*M_PI*8.0*del_t)*zmp_measured_ + 1/(1+2*M_PI*8.0*del_t)*zmp_measured_LPF_;

    // cout << rd_.ContactForce_FT_raw(2) << "," << rd_.ContactForce_FT_raw(8) << endl;
}

void CustomController::calculateFootStepTotal()
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


  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
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
    Kp(0) = 1800.0; Kd(0) = 70.0; // Left Hip yaw
    Kp(1) = 2100.0; Kd(1) = 90.0;// Left Hip roll
    Kp(2) = 2100.0; Kd(2) = 90.0;// Left Hip pitch
    Kp(3) = 2100.0; Kd(3) = 90.0;// Left Knee pitch
    Kp(4) = 1800.0; Kd(4) = 80.0;// Left Ankle pitch
    Kp(5) = 1800.0; Kd(5) = 80.0;// Left Ankle roll

    Kp(6) = 1800.0; Kd(6) = 70.0;// Right Hip yaw
    Kp(7) = 2100.0; Kd(7) = 90.0;// Right Hip roll
    Kp(8) = 2100.0; Kd(8) = 90.0;// Right Hip pitch
    Kp(9) = 2100.0; Kd(9) = 90.0;// Right Knee pitch
    Kp(10) = 1800.0; Kd(10) = 80.0;// Right Ankle pitch
    Kp(11) = 1800.0; Kd(11) = 80.0;// Right Ankle roll

    Kp(12) = 2200.0; Kd(12) = 90.0;// Waist yaw
    Kp(13) = 2200.0; Kd(13) = 90.0;// Waist pitch
    Kp(14) = 2200.0; Kd(14) = 90.0;// Waist roll
        
    Kp(15) = 1600.0; Kd(15) = 70.0;
    Kp(16) = 1600.0; Kd(16) = 70.0;
    Kp(17) = 1600.0; Kd(17) = 70.0;
    Kp(18) = 1600.0; Kd(18) = 70.0;
    Kp(19) = 800.0; Kd(19) = 40.0; 
    Kp(20) = 800.0; Kd(20) = 40.0;
    Kp(21) = 800.0; Kd(21) = 40.0; // Left Wrist
    Kp(22) = 800.0; Kd(22) = 40.0; // Left Wrist
   
    Kp(23) = 800.0; Kd(23) = 40.0; // Neck
    Kp(24) = 800.0; Kd(24) = 40.0; // Neck

    Kp(25) = 1600.0; Kd(25) = 70.0; 
    Kp(26) = 1600.0; Kd(26) = 70.0;
    Kp(27) = 1600.0; Kd(27) = 70.0;
    Kp(28) = 1600.0; Kd(28) = 70.0;
    Kp(29) = 800.0; Kd(29) = 40.0;
    Kp(30) = 800.0; Kd(30) = 40.0;
    Kp(31) = 800.0; Kd(31) = 40.0; // Right Wrist
    Kp(32) = 800.0; Kd(32) = 40.0; // Right Wrist
}

void CustomController::addZmpOffset()
{ 
  double lfoot_zmp_offset_, rfoot_zmp_offset_;
  
  lfoot_zmp_offset_ = -0.02;
  rfoot_zmp_offset_ = 0.02;

  foot_step_support_frame_offset_ = foot_step_support_frame_;
  

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

  //if(current_step_num_ == 0)
  //{ MJ_graph << ref_zmp_(walking_tick_mj,0) << "," << ref_zmp_(walking_tick_mj,1)  << endl; }
  //else
  //{ MJ_graph << ref_zmp_(walking_tick_mj - t_start_, 0) + current_step_num_*0 << "," << ref_zmp_(walking_tick_mj - t_start_,1)  << endl; }
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
      if(i < 0.5*hz_) 
      {
        ref_zmp_(i,0) = com_support_init_(0) ;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else if(i < 1.5*hz_) 
      {
        double del_x = i - 0.5*hz_;
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
        temp_px(i) = supportfoot_support_init_offset_(0);
        temp_py(i) = supportfoot_support_init_offset_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_ ) //1.05 ~ 1.15초 , 210 ~ 230 tick 
      {
        temp_px(i) = supportfoot_support_init_offset_(0) + Kx2 / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
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
        cout << preview_x << "," << preview_y << endl;
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
    
    del_ux(0,0) = -(px(0) - px_ref(tick))*Gi(0,0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0,0) = -(py(0) - py_ref(tick))*Gi(0,0) - GX_Y(0) - sum_Gd_py_ref;
    
    UX = UX + del_ux(0,0);
    UY = UY + del_uy(0,0);

    XD = A*preview_x + B*UX;
    YD = A*preview_y + B*UY;

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

    MJ_graph << px_ref(tick) << "," << zmp_measured_LPF_(0) << "," << py_ref(tick) << "," << zmp_measured_LPF_(1) << endl; 
    
    /*if(tick % 5 == 0 )
    {
      MJ_graph << px_ref(tick) << "," << py_ref(tick) << "," << XD(0) << "," << YD(0) << "," << px(0) << "," << py(0) << endl;
    }*/     
}

void CustomController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);  
  
  pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7*(com_desired_(0) - com_support_current_(0)) - 0.01 * zmp_err_(0);
  pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7*(com_desired_(1) - com_support_current_(1)) - 0.01 * zmp_err_(1);
   
  pelv_trajectory_support_.translation()(2) = com_desired_(2);          

  //MJ_graph << com_desired_(1) << "," << com_support_current_(1) << "," << pelv_support_current_.translation()(1) << endl;
       
  Eigen::Vector3d Trunk_trajectory_euler;
  Trunk_trajectory_euler.setZero();

  if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
  { Trunk_trajectory_euler(2) = pelv_support_euler_init_(2); }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
  { Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_mj,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0); }
  else
  { Trunk_trajectory_euler(2) = z_rot/2.0; } 
  //pelv_support_start_.linear();  
  //cout << z_rot *180/M_PI << "," << current_step_num_ << endl;
  
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

    L_D << 0.0 , +0.1025, -0.1025;
    R_D << 0.0 , -0.1025, -0.1025;

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

}

void CustomController::GravityCalculate_MJ()
{
  double grav_gain = 0.0;
   
  if(walking_tick_mj < t_start_ + t_rest_init_ ) 
  {
    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
    Gravity_SSP_.setZero();
  }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ ) // 0.01 s  
  {
    grav_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_, t_start_ + t_rest_init_ + t_double1_, 0.0, 1.0, 0.0, 0.0);

    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_); 
    
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { 
      wbc_.set_contact(rd_, 1, 0);       
      Gravity_SSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);      
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      wbc_.set_contact(rd_, 0, 1);       
      Gravity_SSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);
    }  
  }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_)
  {
    Gravity_DSP_.setZero(); 

    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { 
      wbc_.set_contact(rd_, 1, 0);       
      Gravity_SSP_ = wbc_.gravity_compensation_torque(rd_);             
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      wbc_.set_contact(rd_, 0, 1);       
      Gravity_SSP_ = wbc_.gravity_compensation_torque(rd_);
    }
  }
  else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
  { 
    grav_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ , t_start_ + t_total_ - t_rest_last_ , 0.0, 1.0, 0.0, 0.0);

    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    {   
      wbc_.set_contact(rd_, 1, 1);
      Gravity_DSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);

      wbc_.set_contact(rd_, 1, 0);       
      Gravity_SSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_);            
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      wbc_.set_contact(rd_, 1, 1);
      Gravity_DSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);

      wbc_.set_contact(rd_, 0, 1);       
      Gravity_SSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_);
    }    
  }
  else
  {
    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
    Gravity_SSP_.setZero();
  }

  Gravity_MJ_ = Gravity_DSP_ + Gravity_SSP_ ;
  
  // SSP
  //wbc_.set_contact(rd_, 1, 1);
  //Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);

}
/*
void CustomController::GravityCalculate_MJ()
{
  Gravity_DSP_.setZero();
  Gravity_SSP_.setZero();
  double grav_gain = 0.0;
  // DSP
  if(walking_tick_mj < t_start_ + t_rest_init_ ) 
  {
    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
  }

  else if(walking_tick_mj >= t_start_ + t_rest_init_  && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) //  
  {
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { 
      grav_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ , t_start_ + t_rest_init_ + t_double1_ , 0.0, 1.0, 0.0, 0.0);
      
     if( walking_tick_mj >= t_start_ + t_rest_init_ &&  walking_tick_mj <= t_start_ + t_rest_init_ + t_double1_)
      {
        wbc_.set_contact(rd_, 1, 1);
        Gravity_DSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_);
      }       

      wbc_.set_contact(rd_, 1, 0);       
      Gravity_SSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_); 
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      grav_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ , t_start_ + t_rest_init_ + t_double1_ , 0.0, 1.0, 0.0, 0.0);

      if( walking_tick_mj >= t_start_ + t_rest_init_ &&  walking_tick_mj <= t_start_ + t_rest_init_ + t_double1_)
      {
        wbc_.set_contact(rd_, 1, 1);
        Gravity_DSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_);
      }     
      
      wbc_.set_contact(rd_, 0, 1);       
      Gravity_SSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);
    }  
  }
  else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ )
  {
    if(foot_step_(current_step_num_,6) == 1) // 왼발 지지
    { 
      grav_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ , t_start_ + t_total_ - t_rest_last_ , 0.0, 1.0, 0.0, 0.0);

      wbc_.set_contact(rd_, 1, 1);
      Gravity_DSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);      
      
      wbc_.set_contact(rd_, 1, 0);       
      Gravity_SSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_);
      
            
    }
    else if(foot_step_(current_step_num_,6) == 0) // 오른발 지지
    {
      grav_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ , t_start_ + t_total_ - t_rest_last_ , 0.0, 1.0, 0.0, 0.0);

      wbc_.set_contact(rd_, 1, 1);
      Gravity_DSP_ = grav_gain * wbc_.gravity_compensation_torque(rd_);      
      
      wbc_.set_contact(rd_, 0, 1);       
      Gravity_SSP_ = (1.0 - grav_gain) * wbc_.gravity_compensation_torque(rd_);
    }
  }
  else
  {
    wbc_.set_contact(rd_, 1, 1);
    Gravity_DSP_ = wbc_.gravity_compensation_torque(rd_);
  }

  Gravity_MJ_ = Gravity_DSP_ + Gravity_SSP_ ;
  
  // SSP
  //wbc_.set_contact(rd_, 1, 1);
  //Gravity_MJ_ = wbc_.gravity_compensation_torque(rd_);

}
*/
void CustomController::parameterSetting()
{
    target_x_ = 6.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.15;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    t_rest_init_ = 0.08*hz_; 
    t_rest_last_ = 0.08*hz_;  
    t_double1_ = 0.02*hz_;
    t_double2_ = 0.02*hz_; 
    t_total_= 1.0*hz_;  
    t_temp_ = 3.0*hz_;
    t_last_ = t_total_ + t_temp_ ;
    t_start_ = t_temp_ + 1 ;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
    foot_height_ = 0.06; // 제자리 보행 파라미터 4cm // 전진 5cm
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
     walking_enable_ = false;
   }
   walking_tick_mj ++;
}


void CustomController::hip_compensator()
{  
  double left_hip_roll = -0.00*DEG2RAD, right_hip_roll = -0.00*DEG2RAD, left_hip_roll_first = -0.0*DEG2RAD, right_hip_roll_first = -0.0*DEG2RAD, 
  left_hip_pitch = 1.0*DEG2RAD, right_hip_pitch = 1.0*DEG2RAD, left_hip_pitch_first = 1.0*DEG2RAD, right_hip_pitch_first = 1.0*DEG2RAD,
      left_hip_roll_temp = 0.0, right_hip_roll_temp = 0.0, left_hip_pitch_temp = 0.0, right_hip_pitch_temp = 0.0, temp_time = 0.1*hz_;


  if (current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_hip_roll_first, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_hip_pitch_first, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_roll_first, 0.0, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_pitch_first, 0.0, 0.0, 0.0);
      }      
      else
      { left_hip_roll_temp = 0; left_hip_pitch_temp = 0; }      
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_hip_roll_first, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_hip_pitch_first, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_roll_first, 0.0, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_pitch_first, 0.0, 0.0, 0.0);
      }        
      else
      { right_hip_roll_temp = 0; right_hip_pitch_temp = 0; }
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
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_roll, 0.0, 0.0, 0.0);
        left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_pitch, 0.0, 0.0, 0.0);
      }      
      else
      { left_hip_roll_temp = 0; left_hip_pitch_temp = 0; }      
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_roll, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_pitch, 0.0, 0.0);
      }      
      else if(walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
      {
        right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_roll, 0.0, 0.0, 0.0);
        right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_pitch, 0.0, 0.0, 0.0);
      }        
      else
      { right_hip_roll_temp = 0; right_hip_pitch_temp = 0; }
    }    
  }  
  
  ref_q_(1) = ref_q_(1) - left_hip_roll_temp;
  ref_q_(7) = ref_q_(7) + right_hip_roll_temp;
  ref_q_(2) = ref_q_(2) - left_hip_pitch_temp;
  ref_q_(8) = ref_q_(8) - right_hip_pitch_temp;
}

void CustomController::Compliant_control(Eigen::Vector12d desired_leg_q)
{     
  Eigen::Vector12d current_u; 
  double del_t = 0.0, Kp = 0.0;
  del_t = 1/hz_; Kp = 20.0;

  if(walking_tick_mj == 0)  
  {
    for(int i = 0; i < 12; i++)
    { DOB_IK_output_b_(i) = rd_.q_(i); DOB_IK_output_(i) = rd_.q_(i); current_u(i) = rd_.q_(i); }    
  }
  
  if(walking_tick_mj > 0)
  {
    for (int i = 0; i < 12; i++)
    { 
      current_u(i) = (rd_.q_(i) - (1 - Kp*del_t)*rd_.q_prev_MJ_(i)) / (Kp*del_t);
    }
  }
  
  Eigen::Vector12d d_hat;    
  d_hat = current_u - DOB_IK_output_b_ ; 
  
  if(walking_tick_mj == 0)
    d_hat_b = d_hat;

  d_hat = 0.97*d_hat_b + 0.03*d_hat;  

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

void CustomController::computePlanner()
{
}
