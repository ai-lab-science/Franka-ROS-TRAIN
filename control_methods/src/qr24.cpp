#include "qr24.h"

#include <string>
#include <iostream>
#include <fstream>

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

    // Invert M_i
    MatrixXd M_i_inv(4,4);
    M_i_inv = M[i].inverse();

    // A_i
    MatrixXd A_i(12,24);
    A_i = MatrixXd::Zero(12,24);
    for (int ii=0; ii<4; ii++) {
      for (int jj=0; jj<3; jj++) {
        A_i.block(ii*3, jj*3, 3, 3) = M_i_inv.block(0,0,3,3) * N[i](ii,jj);
      }
    }
    A_i.block(9,9,3,3) = M_i_inv.block(0,0,3,3);
    A_i.block(0,12,12,12) = -MatrixXd::Identity(12, 12);

    // b_i
    VectorXd b_i(12);
    b_i = VectorXd::Zero(12);
    b_i.segment(9,3) = -M_i_inv.block(0,3,3,1);

    // Put A_i and b_i in A and b
    A.block(12*i,0,12,24) = A_i;
    b.segment(12*i,12) = b_i;

  }

  // Solve with QR decomposition
  VectorXd w(24);
  w = A.colPivHouseholderQr().solve(b);

  // Store solution in matrices
  X = Matrix4d::Zero();
  X(0,0) = w(0); X(1,0) = w(1); X(2,0) = w(2);
  X(0,1) = w(3); X(1,1) = w(4); X(2,1) = w(5);
  X(0,2) = w(6); X(1,2) = w(7); X(2,2) = w(8);
  X(0,3) = w(9); X(1,3) = w(10); X(2,3) = w(11);
  X(3,3) = 1;

  Y = Matrix4d::Zero();
  Y(0,0) = w(12); Y(1,0) = w(13); Y(2,0) = w(14);
  Y(0,1) = w(15); Y(1,1) = w(16); Y(2,1) = w(17);
  Y(0,2) = w(18); Y(1,2) = w(19); Y(2,2) = w(20);
  Y(0,3) = w(21); Y(1,3) = w(22); Y(2,3) = w(23);
  Y(3,3) = 1;

  // Print the results
  std::cout << "\n" << "The matrix X:\n" << X << std::endl;
  std::cout << "\n" << "The matrix Y:\n" << Y << std::endl;

  // Save the data
  std::ofstream file("/home/rob/train_ws/src/train_methods/control_methods/config/test.txt");
  if (file.is_open()) {
    file << X << "\n";
    file << Y << "\n";
    file << "--- OptiTrack ---" << "\n";
    for (int i=0; i<measurements; i++) file << M[i] << "\n";
    file << "--- EndEffektor ---" << "\n";
    for (int i=0; i<measurements; i++) file << N[i] << "\n";
    std::cout << "Saved data!" << std::endl;
  }  

}
