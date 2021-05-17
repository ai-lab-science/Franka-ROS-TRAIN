#include "qr24.h"


// The constructor
QR24::QR24() {

  measurements = 0;
  calibrated = false;

}


// Store a measurement
bool QR24::storeMeasurement(geometry_msgs::PoseStamped calObj, geometry_msgs::PoseStamped endeffector) {

  if (measurements >= ARRAY_SIZE) {

    return false;

  } else {

    // Initialise with zeros
    M[measurements] = Matrix4d::Zero();
    N[measurements] = Matrix4d::Zero();

    // Define Eigen3 Quaternions
    Quaterniond qCal(calObj.pose.orientation.w, calObj.pose.orientation.x, calObj.pose.orientation.y, calObj.pose.orientation.z);
    Quaterniond qEnd(endeffector.pose.orientation.w, endeffector.pose.orientation.x, endeffector.pose.orientation.y, endeffector.pose.orientation.z);

    // Get the rotation matrices
    Matrix3d rCal = qCal.toRotationMatrix();
    Matrix3d rEnd = qEnd.toRotationMatrix();

    // Fill in the rotation matrix
    M[measurements].block(0,0,3,3) = rEnd;
    N[measurements].block(0,0,3,3) = rCal;

    // Fill in the translations
    Vector4d tCal;
    tCal << calObj.pose.position.x, calObj.pose.position.y, calObj.pose.position.z, 1;
    M[measurements].block(0,3,4,1) = tCal;
    Vector4d tEnd;
    tEnd << endeffector.pose.position.x, endeffector.pose.position.y, endeffector.pose.position.z, 1;
    N[measurements].block(0,3,4,1) = tEnd;

    // Count up and return true
    measurements++;
    return true;

  }

}


// Calculate the Calibration
bool QR24::calculateCalibration() {

  // Define A matrix and b vector
  MatrixXd A(12*measurements,24);
  VectorXd b(12*measurements);

  // Fill the A matrix and the b vector
  for (int i=0; i<measurements; i++) {

    // A_i
    MatrixXd A_i(12,24);
    A_i = MatrixXd::Zero(12,24);
    for (int ii=0; ii<4; ii++) {
      for (int jj=0; jj<3; jj++) {
        A_i.block(ii*3, jj*3, 3, 3) = M[i].block(0,0,3,3) * N[i](ii,jj);
      }
    }
    A_i.block(9,9,3,3) = M[i].block(0,0,3,3);
    A_i.block(0,12,12,12) = -MatrixXd::Identity(12, 12);

    // b_i
    VectorXd b_i(12);
    b_i = VectorXd::Zero(12);
    b_i.segment(9,3) = -M[i].block(0,3,3,1);

    // Put A_i and b_i in A and b
    A.block(12*i,0,12,24) = A_i;
    b.segment(12*i,12) = b_i;

  }

  // Solve with QR decomposition
  VectorXd w(24);
  w = A.colPivHouseholderQr().solve(b);
  for (int i=0; i<24; i++){
    ROS_INFO("%f", w(i));
  } 

}
