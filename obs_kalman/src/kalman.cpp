#include "obs_kalman/kalman.hpp"

void Kalman::Init_Par(Eigen::VectorXd& x, Eigen::MatrixXd& P, Eigen::MatrixXd& R, Eigen::MatrixXd& Q,
                      Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& H, Eigen::VectorXd& u)
{
  m_x = x;//状态
  m_u = u;//控制输入

  m_A = A;//状态转移阵
  m_B = B;//控制阵
  m_H = H;//观测阵

  m_P = P;//状态协方差
  m_Q = Q;//过程噪声协方差 
  m_R = R;//测量噪声协方差
  
  
}

void Kalman::Predict_State()//计算先验
{
  m_x = m_A * m_x + m_B * m_u;
}

void Kalman::Predict_Cov()//计算先验误差协方差
{
  m_P = m_A * m_P * m_A.transpose() + m_Q;
}


Eigen::MatrixXd Kalman::Cal_Gain()//计算卡尔曼增益
{
  return m_P * m_H.transpose() * (m_H * m_P * m_H.transpose() + m_R).inverse();
}

void Kalman::Update_State()//后验估计
{
  Eigen::MatrixXd kal_gain = Cal_Gain();
  Eigen::VectorXd mea_res = m_z - m_H * m_x;
  m_x += kal_gain * mea_res;
}

void Kalman::Update_Cov()//更新误差协方差
{
  Eigen::MatrixXd kal_gain = Cal_Gain();
  m_P = m_P - kal_gain * m_H * m_P;
}