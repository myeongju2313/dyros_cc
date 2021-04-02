#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"

class CustomController
{
public:
  CustomController(DataContainer &dc,RobotData &rd);
  Eigen::VectorQd getControl();

  void taskCommandToCC(TaskCommand tc_);
  
  void computeSlow();
  void computeFast();
  void computePlanner();
  
  DataContainer &dc_;
  RobotData &rd_;
  WholebodyController &wbc_;
  TaskCommand tc;

  //////////////////////////////// Myeong-Ju
  void circling_motion();
  void computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
  void Joint_gain_set_MJ();
  void updateInitialState();
  void updateNextStepTime();
  void parameterSetting();
  void getRobotState();
  void calculateFootStepTotal();
  void calculateFootStepTotal_MJ();
  void supportToFloatPattern();
  void floatToSupportFootstep();
  void GravityCalculate_MJ();
 
  void getZmpTrajectory();
  void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
  void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
  void getComTrajectory();
  void getFootTrajectory();
  void getPelvTrajectory();
  void previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
  Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD);  
  void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
  void addZmpOffset();
  void hip_compensator();
  void Compliant_control(Eigen::Vector12d desired_leg_q);
  
  void SC_err_compen(double x_des, double y_des);

  Eigen::Vector12d pre_motor_q_leg_;
  Eigen::Vector12d current_motor_q_leg_;
  Eigen::Vector12d d_hat_b;
  Eigen::Vector12d DOB_IK_output_b_;
  Eigen::Vector12d DOB_IK_output_;
  Eigen::VectorQd ref_q_;
  Eigen::VectorQd Kp;
  Eigen::VectorQd Kd;
  Eigen::VectorQd desired_q_not_compensated_;
  
  Eigen::VectorQd q_prev_MJ_;

  Eigen::Vector12d q_des;
  
  Eigen::Isometry3d pelv_trajectory_support_; //local frame
  
  Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
  Eigen::Isometry3d lfoot_trajectory_support_;
  Eigen::Vector3d rfoot_trajectory_euler_support_;
  Eigen::Vector3d lfoot_trajectory_euler_support_;

  Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame
  Eigen::Isometry3d rfoot_trajectory_float_;
  Eigen::Isometry3d lfoot_trajectory_float_;

  Eigen::Vector3d pelv_support_euler_init_;
  Eigen::Vector3d lfoot_support_euler_init_;
  Eigen::Vector3d rfoot_support_euler_init_;

  Eigen::Isometry3d pelv_support_start_;
  Eigen::Isometry3d pelv_support_init_;
  Eigen::Vector2d del_zmp;
  Eigen::Vector2d cp_desired_;
  Eigen::Vector2d cp_measured_;
  Eigen::Vector2d cp_measured_LPF;
  Eigen::Vector3d com_support_init_;
  Eigen::Vector3d com_float_init_;
  Eigen::Vector3d com_float_current_;
  Eigen::Vector3d com_support_current_;
  Eigen::Vector3d com_support_current_dot;
  Eigen::Vector3d com_support_current_LPF;
  Eigen::Vector3d com_float_current_LPF;
  Eigen::Vector3d com_support_current_prev;

  Eigen::Vector3d pelv_rpy_current_;
  Eigen::Isometry3d pelv_yaw_rot_current_from_global_;

  Eigen::Isometry3d pelv_float_current_;
  Eigen::Isometry3d lfoot_float_current_;
  Eigen::Isometry3d rfoot_float_current_;
  Eigen::Isometry3d pelv_float_init_;
  Eigen::Isometry3d lfoot_float_init_;
  Eigen::Isometry3d rfoot_float_init_;
  double wn = 0;

  Eigen::Vector2d sc_err_before;
  Eigen::Vector2d sc_err_after;
  Eigen::Vector2d SC_com;
  Eigen::Vector2d sc_err;

  Eigen::Vector12d sc_joint_before;
  Eigen::Vector12d sc_joint_after;
  Eigen::Vector12d SC_joint;
  Eigen::Vector12d sc_joint_err;

  double walking_end_flag = 0;
  
  Eigen::Isometry3d supportfoot_float_current_; 

  Eigen::Isometry3d pelv_support_current_;
  Eigen::Isometry3d lfoot_support_current_;
  Eigen::Isometry3d rfoot_support_current_;

  Eigen::Isometry3d lfoot_support_init_;
  Eigen::Isometry3d rfoot_support_init_;
  
  Eigen::Vector6d supportfoot_support_init_offset_;
  Eigen::Vector6d supportfoot_float_init_;
  Eigen::Vector6d supportfoot_support_init_;
  Eigen::Vector6d swingfoot_float_init_;
  Eigen::Vector6d swingfoot_support_init_;

  Eigen::MatrixXd ref_zmp_;

  Eigen::Vector3d xs_;
  Eigen::Vector3d ys_;
  Eigen::Vector3d xd_;
  Eigen::Vector3d yd_; 
  Eigen::Vector3d preview_x, preview_y, preview_x_b, preview_y_b;

  Eigen::MatrixXd Gi_;
  Eigen::MatrixXd Gx_;
  Eigen::VectorXd Gd_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd B_;
  Eigen::MatrixXd C_;

  Eigen::VectorQd Gravity_MJ_;
  Eigen::VectorQd Gravity_DSP_;
  Eigen::VectorQd Gravity_DSP_last_;
  Eigen::VectorQd Gravity_SSP_;
  Eigen::VectorQd Gravity_SSP_last_;
  Eigen::VectorQd q_dot_LPF_MJ;

  Eigen::Vector6d r_ft_;
  Eigen::Vector6d l_ft_;
  Eigen::Vector2d zmp_measured_;
  Eigen::Vector2d zmp_err_;
  Eigen::Vector2d zmp_measured_LPF_;

  double del_t = 0.0005;
  double xi_;
  double yi_;
  double zc_;

  double t_last_;
  double t_start_;
  double t_start_real_;
  double t_temp_;  
  double t_rest_init_;
  double t_rest_last_;
  double t_double1_;
  double t_double2_;
  double t_total_;
  double foot_height_;
  double total_step_num_;
  double current_step_num_;

  double step_length_x_;
  double step_length_y_;
  double target_theta_;
  double target_x_;
  double target_y_;
  double target_z_;
  double com_height_;
  int is_right_foot_swing_;

  double zmp_start_time_;
  double UX_, UY_; 
  Eigen::Vector3d com_desired_;
  Eigen::MatrixXd foot_step_;
  Eigen::MatrixXd foot_step_support_frame_;
  Eigen::MatrixXd foot_step_support_frame_offset_;

  Eigen::VectorQd contact_torque_MJ;
  Eigen::VectorQd Initial_ref_q_;
  Eigen::VectorQd Initial_ref_q_walk_;
  bool walking_enable_ ;

  Eigen::VectorQd q_mj;
  Eigen::VectorQd q_mj_prev;
private:
  Eigen::VectorQd ControlVal_;
  
  //////////////////////////////// Myeong-Ju
  unsigned int walking_tick_mj = 0;
  unsigned int initial_flag = 0;
  const double hz_ = 2000.0;
};


