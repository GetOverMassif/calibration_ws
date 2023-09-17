#include <iostream>
#include "Eigen/Geometry"

const int WINDOW_SIZE = 10;

Eigen::Vector3d G{0.0, 0.0, 9.8};

// # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)
double TD = 0.0;
extern int USE_IMU = true;
extern bool MULTIPLE_THREAD = true;

extern int ESTIMATE_EXTRINSIC = 2;