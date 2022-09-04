/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "Controllers/PositionVelocityEstimator.h"

/*!
 * Initialize the state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {
  T dt = this->_stateEstimatorData.parameters->controller_dt;
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  //todo 状态变量 x = [p  v  p1  p2  p3  p4]
  //  pk+1 = pk + vk*dt
  //  vk+1 = vk + ak*dt    ==>    xk+1 = Akxk + Bkuk + vk
  //  pi,k+1 = pi,k
  //       
  //         I_3      I_3*dt   0_3x12           0_3x3
  //  Ak  =  0_3x3    I_3      0_3x12    Bk =   I_3*dt
  //         0_12x3   0_12x3   I_12             0_12x3
  //构造 A矩阵
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
  //构造 B矩阵
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = T(1);
  _C(26, 14) = T(1);
  _C(25, 11) = T(1);
  _C(24, 8) = T(1);
  _P.setIdentity();
  _P = T(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  _R0.setIdentity();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run() {

  //TODO  位置和速度估计
  T process_noise_pimu =
      this->_stateEstimatorData.parameters->imu_process_noise_position;
  T process_noise_vimu =
      this->_stateEstimatorData.parameters->imu_process_noise_velocity;
  T process_noise_pfoot =
      this->_stateEstimatorData.parameters->foot_process_noise_position;
  T sensor_noise_pimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_position;
  T sensor_noise_vimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
  T sensor_noise_zfoot =
      this->_stateEstimatorData.parameters->foot_height_sensor_noise;

  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<T> g(0, 0, T(-9.81));
  Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  // in old code, Rbod * se_acc + g
  //         ax     0
  //    u =  ay  +  0 
  //         az    -9.8
  Vec3<T> a = this->_stateEstimatorData.result->aWorld + g; 
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();
  Vec3<T> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];
  //todo 求观测方程zk+1 = Hkxk + wk  zk+1中的各个分量 zk = [bp1 bp2 bp3 bp4 ov1 ov2 ov3 ov4 oz1 oz2 oz3]
  //b代表在机器人坐标系下  o在世界坐标系下 
  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
    Quadruped<T>& quadruped =
        *(this->_stateEstimatorData.legControllerData->quadruped);
    Vec3<T> ph = quadruped.getHipLocation(i);  // hip positions relative to CoM
    // hw_i->leg_controller->leg_datas[i].p; 
    //1.全局坐标系下足端相对与机身的位置由腿i正解加上臀部位置，通过机身到全局坐标系的旋转矩阵转化得到
    //2.观测量中的速度应该表示为足端相对于瞬间机身质心处惯性坐标系的速度，
    //  如果是足端速度在全局坐标系下速度，应该需要加上机身速度。该速度通过关节速度算出足端末端速度vrel,再通过机身角速度w和旋转矩阵转换到惯性坐标系中
    //足端相对与质心的位置
    Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
    // hw_i->leg_controller->leg_datas[i].v;
    Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v;  
    // op1 = op + oRb*bp1  ==> -oRb*bp1 = op - op1
    Vec3<T> p_f = Rbod * p_rel;

  
    Vec3<T> dp_f =
        Rbod *
        (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    T trust = T(1);
    T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));
    //T trust_window = T(0.25);
    T trust_window = T(0.2);

    // 当腿处于摆动相时，提高腿的位置预测误差方差100倍，
    // 同时提高观测误差中腿的速度和高度z的观测误差100倍，即在摆动时，腿部位置以观测为准，
    // 腿部速度和高度z以预测值为准。具体过程如下，对每条腿引入置信度trust和腿支撑状态相位phase,
    // 摆腿时phase=0，支撑时pahse由0变化到1。
    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }
    //T high_suspect_number(1000);
    T high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
    // 当腿处于摆动状态时，用机身速度表示全局坐标系下腿末端相对机身质心速度，不准，
    // 所以在摆动状态下将速度观测值的协方差扩大100倍，以预测值为准。在支撑时，质心速度和vf大小相等，
    // 符号相反，由于观测矩阵中系数取正，所以这里用vf的负值。

    Q.block(qindex, qindex, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i) = trust;

  

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<T, 28, 1> y;
  //构造 zk+1
  y << _ps, _vs, pzs;
  //预测过程
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, 28, 1> ey = y - yModel;
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }

  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody =
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run() {
  this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
  this->_stateEstimatorData.result->vWorld =
      this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
