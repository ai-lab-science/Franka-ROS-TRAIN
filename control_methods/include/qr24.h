#ifndef __qr24_h
#define __qr24_h

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

#define ARRAY_SIZE 10

using namespace Eigen;

class QR24 {

private:

  // Counter and Arrays for measurements
  int measurements;
  Matrix4d M[ARRAY_SIZE];
  Matrix4d N[ARRAY_SIZE];

  // The resulting calibration matrices and if calibration has been performed
  bool calibrated;
  Matrix4d X;
  Matrix4d Y;

public:

  // The constructor
  QR24();

  // Store a measurement
  bool storeMeasurement(geometry_msgs::PoseStamped calObj, geometry_msgs::PoseStamped endeffector);

  // Calibration procedure
  bool calculateCalibration();

};

#endif
