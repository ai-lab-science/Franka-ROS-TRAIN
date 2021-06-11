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
bool QR24::storeMeasurement(geometry_msgs::PoseStamped baseToEndeffector, geometry_msgs::PoseStamped trackingToMarker) {

  if (measurements >= ARRAY_SIZE) {

    return false;

  } else {

    // For Debugging
    QuatM[measurements] << baseToEndeffector.pose.orientation.w, baseToEndeffector.pose.orientation.x, baseToEndeffector.pose.orientation.y, baseToEndeffector.pose.orientation.z;
    QuatN[measurements] << trackingToMarker.pose.orientation.w, trackingToMarker.pose.orientation.x, trackingToMarker.pose.orientation.y, trackingToMarker.pose.orientation.z;
    PoseM[measurements] << baseToEndeffector.pose.position.x, baseToEndeffector.pose.position.y, baseToEndeffector.pose.position.z;
    PoseN[measurements] << trackingToMarker.pose.position.x, trackingToMarker.pose.position.y, trackingToMarker.pose.position.z;

    // Initialise with zeros
    M[measurements] = Matrix4d::Zero();
    N[measurements] = Matrix4d::Zero();

    // Define Eigen3 Quaternions
    Quaterniond qM(baseToEndeffector.pose.orientation.w, baseToEndeffector.pose.orientation.x, baseToEndeffector.pose.orientation.y, baseToEndeffector.pose.orientation.z);
    Quaterniond qN(trackingToMarker.pose.orientation.w, trackingToMarker.pose.orientation.x, trackingToMarker.pose.orientation.y, trackingToMarker.pose.orientation.z);

    // Get the rotation matrices
    Matrix3d rM = qM.toRotationMatrix();
    Matrix3d rN = qN.toRotationMatrix();

    // Fill in the rotation matrix
    M[measurements].block(0,0,3,3) = rM;
    N[measurements].block(0,0,3,3) = rN;

    // Fill in the translations
    Vector4d tM;
    tM << baseToEndeffector.pose.position.x, baseToEndeffector.pose.position.y, baseToEndeffector.pose.position.z, 1;
    M[measurements].block(0,3,4,1) = tM;
    Vector4d tN;
    tN << trackingToMarker.pose.position.x, trackingToMarker.pose.position.y, trackingToMarker.pose.position.z, 1;
    N[measurements].block(0,3,4,1) = tN;

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

    // Ralf's Implemetation (Robotik Internship 01, Uni Lübeck)
    // A_i
    MatrixXd A_i(12,24);
    A_i = MatrixXd::Zero(12,24);
    for (int ii=0; ii<4; ii++) {
      A_i.block(ii*3, ii*3, 3, 3) = M[i].block(0,0,3,3);

      A_i.block(ii*3, 12, 1, 3) = -N[i].block(0,ii,3,1).transpose();
      A_i.block(ii*3+1, 16, 1, 3) = -N[i].block(0,ii,3,1).transpose();
      A_i.block(ii*3+2, 20, 1, 3) = -N[i].block(0,ii,3,1).transpose();
    }
    A_i(9,15) = -1;
    A_i(10,19) = -1;
    A_i(11,23) = -1;

    // b_i
    VectorXd b_i(12);
    b_i = VectorXd::Zero(12);
    b_i.segment(9,3) = -M[i].block(0,3,3,1);

    // Put A_i and b_i in A and b
    A.block(12*i,0,12,24) = A_i;
    b.segment(12*i,12) = b_i;

  }

  std::cout << "\n" << "The matrix A:\n" << A << std::endl;
  std::cout << "\n" << "The vector b:\n" << b << std::endl;

  // Solve with QR decomposition
  VectorXd w(24);
  w = A.fullPivLu().solve(b);

  // Store solution in matrices
  X = Matrix4d::Zero();
  X.block(0,0,3,1) = w.segment(0,3);
  X.block(0,1,3,1) = w.segment(3,3);
  X.block(0,2,3,1) = w.segment(6,3);
  X.block(0,3,3,1) = w.segment(9,3);
  X(3,3) = 1;

  Y = Matrix4d::Zero();
  Y.block(0,0,1,4) = w.segment(12,4).transpose();
  Y.block(1,0,1,4) = w.segment(16,4).transpose();
  Y.block(2,0,1,4) = w.segment(20,4).transpose();
  Y(3,3) = 1;

  // Print the results
  std::cout << "\n" << "The matrix X:\n" << X << std::endl;
  std::cout << "\n" << "The matrix Y:\n" << Y << std::endl;

  // Save the data
  std::ofstream fileSolution("/home/rob/train_ws/src/train_methods/control_methods/config/solution.txt");
  if (fileSolution.is_open()) {
    fileSolution << "X: Endeffektor to Marker \n";
    fileSolution << X << "\n\n";
    fileSolution << "Y: Robot's base to Tracking System \n";
    fileSolution << Y << "\n\n";
    fileSolution << "M: Robot's base to endeffector \n";
    for (int i=0; i<measurements; i++) fileSolution << M[i] << "\n\n";
    fileSolution << "N: Tracking System to Marker \n";
    for (int i=0; i<measurements; i++) fileSolution << N[i] << "\n\n";
    std::cout << "Saved solution data!" << std::endl;
  }  

  // Save the data for Debugging
  std::ofstream fileDebug("/home/rob/train_ws/src/train_methods/control_methods/config/debug.txt");
  if (fileDebug.is_open()) {
    fileDebug << "Quat M: Robot's base to endeffector (w,x,y,z) \n";
    for (int i=0; i<measurements; i++) fileDebug << QuatM[i] << "\n\n";
    fileDebug << "Quat N: Tracking System to Marker (w,x,y,z) \n";
    for (int i=0; i<measurements; i++) fileDebug << QuatN[i] << "\n\n";
    fileDebug << "Pose M: Robot's base to endeffector (x,y,z) \n";
    for (int i=0; i<measurements; i++) fileDebug << PoseM[i] << "\n\n";
    fileDebug << "Pose N: Tracking System to Marker (x,y,z) \n";
    for (int i=0; i<measurements; i++) fileDebug << PoseN[i] << "\n\n";
    std::cout << "Saved debug data!" << std::endl;
  }  
}
