#pragma once

#include "Eigen/Dense"

class CameraOdomErr
  {
  private:
    Eigen::Quaterniond m_q1, m_q2;
    Eigen::Vector3d m_t1,m_t2;

  public:
  //odom cam
    CameraOdomErr(Eigen::Quaterniond q1, Eigen::Vector3d t1,    
      Eigen::Quaterniond q2, Eigen::Vector3d t2)   
    : m_q1(q1),m_q2(q2), m_t1(t1),m_t2(t2)
    { }

  template<typename T>
    bool operator() (const T* const q4x1 , const T* const t3x1, T* residuals) const
    { 
      Eigen::Quaternion<T> qoc( q4x1[0], q4x1[1],q4x1[2],q4x1[3]);
      Eigen::Matrix<T,3,1> toc;
      toc<<t3x1[0],t3x1[1], T(0);

      Eigen::Quaternion<T> q_odo = m_q1.cast<T>();
      Eigen::Matrix<T,3,1>  t_odo =  m_t1.cast<T>();
      Eigen::Quaternion<T> q_cc = m_q2.cast<T>();
      Eigen::Matrix<T, 3,1> t_cc = m_t2.cast<T>();

      Eigen::Matrix<T,3,3> R_odo = q_odo.toRotationMatrix();
      Eigen::Matrix<T,3,3> Roc = qoc.toRotationMatrix();

      Eigen::Matrix<T, 3,1> t_err = (R_odo - Eigen::Matrix<T,3,3>::Identity() ) * toc - (Roc * t_cc) + t_odo;

    //  q is unit quaternion,   q.inv() = q.conjugate();
    //  q_odo * q_oc = q_oc * q_cc  -->   q_oc.conjugate() * q_odo * q_oc * q_cc.conjugate() = 0;
      //Eigen::Quaternion<T> q_err = q_odo * qoc - qoc * q_cc;
      // Eigen::Matrix<T,3,3> R_err = q_err.toRotationMatrix();

      // T roll, pitch, yaw;
      // mat2RPY(R_err,roll, pitch,yaw);

      residuals[0] = t_err[0];
      residuals[1] = t_err[1];
      residuals[2] = t_err[2];
      residuals[3] = (q_odo * qoc).w() - (qoc * q_cc).w();
      residuals[4] = (q_odo * qoc).x() - (qoc * q_cc).x();
      residuals[5] = (q_odo * qoc).y() - (qoc * q_cc).y();
      residuals[6] = (q_odo * qoc).z() - (qoc * q_cc).z();
      
      return true;
    }
  };