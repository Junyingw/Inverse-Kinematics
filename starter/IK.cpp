#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Spring 2019
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{ 
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints are subject to IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles
// Output: handlePositions



template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.

  int totalNum = fk.getNumJoints();
 
  vector<Mat3<real>> getEulerMat3(totalNum);
  vector<Vec3<real>> getJointOrientVec(totalNum);
  vector<Mat3<real>> getJointOrientMat3(totalNum);
  vector<Mat3<real>> getLocalTransMat3(totalNum);
  vector<Mat3<real>> getGlobalTransMat3(totalNum);
  vector<Vec3<real>> getGlobalTranslation(totalNum);
  vector<Vec3<real>> getLocalTranslation(totalNum);

  // M(Local) = M(JointOritation) * M(EulertoRotation) *  M(Translations) 
  // globalTransform = parentGlobalTransform * localTransform

  // get local transform positions
  for(int i=0; i<totalNum; i++)
  {
    // convert Euler to matrixs 3d
    adouble temptArray1[3]= {0.0,0.0,0.0};
    double temptArray2[3]= {0.0,0.0,0.0};
    double temptArray3[3]= {0.0,0.0,0.0};
    adouble temptArray4[3]= {0.0,0.0,0.0};

    // get joint oritation Euler angles
    fk.getJointOrient(i).convertToArray(temptArray2);
    getJointOrientVec[i] = Vec3<adouble>(temptArray2);
    getJointOrientVec[i].convertToArray(temptArray4);

    temptArray1[0] = eulerAngles[3*i+0];
    temptArray1[1] = eulerAngles[3*i+1];
    temptArray1[2] = eulerAngles[3*i+2];

    getEulerMat3[i] = Euler2Rotation(temptArray1, fk.getJointRotateOrder(i));
    getJointOrientMat3[i] = Euler2Rotation(temptArray4,XYZ);

    // get the local transformation matrixs 3d
    getLocalTransMat3[i] = getJointOrientMat3[i]*getEulerMat3[i];

    // get the local translation vectors
    fk.getJointRestTranslation(i).convertToArray(temptArray3);
    getLocalTranslation[i] = Vec3<adouble>(temptArray3);

    memset(&temptArray1, 0, sizeof(adouble[3]));
    memset(&temptArray2, 0, sizeof(double[3]));
    memset(&temptArray3, 0, sizeof(double[3]));

  }
  
  // get handle global position
  for(int i=0; i<totalNum; i++)
  {
    int t = fk.getJointUpdateOrder(i);
    //cout<<t<<endl;

    // if root 
    if(fk.getJointParent(t)==-1) 
    {
      getGlobalTransMat3[i] = getLocalTransMat3[t];
      getGlobalTranslation[i] = getLocalTranslation[t];
    }
    else 
    { 
      multiplyAffineTransform4ds(getGlobalTransMat3[fk.getJointParent(t)], getGlobalTranslation[fk.getJointParent(t)],
      getLocalTransMat3[t], getLocalTranslation[t], getGlobalTransMat3[t], getGlobalTranslation[t]); 
    } 
    // look up to find if the current joint is IK joint
    for(int j=0; j<numIKJoints; j++)
    {
      // get global handle positions
      if(t==IKJointIDs[j])
      {
        adouble temptArray[3] = {0.0,0.0,0.0};
      
        getGlobalTranslation[t].convertToArray(temptArray);

        handlePositions[3*j+0] = temptArray[0];
        handlePositions[3*j+1] = temptArray[1];
        handlePositions[3*j+2] = temptArray[2];

        memset(&temptArray, 0, sizeof(adouble[3]));
      }
    }
  }

}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp 

  int n = FKInputDim; // input dimension is n
  int m = FKOutputDim; // output dimension is m

  // start tracking computation with ADOL-C
  trace_on(adolc_tagID);

  // define the input of the function f
  // input is Euler Angles
  vector<adouble> x(n); 

  for(int i = 0; i<n; i++)
    x[i] <<= 0.0;

  vector<adouble> y(m); 

  // use forwardKinematicsFunction as the computed function f
  forwardKinematicsFunction(numIKJoints,IKJointIDs,(*fk),x,y);
  
  // define the output of the function f
  // output is handle global positions
  vector<double> output(m);
  for(int i = 0; i < m; i++)
    y[i] >>= output[i];

  // ADOL-C tracking finished
  trace_off(); 
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.

  // std::string sep = "\n----------------------------------------\n";
  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  // You may find the following helpful:
  int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

  int n = FKInputDim; // input dimension is n
  int m = FKOutputDim; // output dimension is m

  // define the alpha value, the larger the alpha, the more stable the method
  double alpha = 0.01;

  // define the change of the handle positions, using deltaPos vector
  vector<Vec3d> deltaPos(numIKJoints);
  vector<Vec3d> getOriginalPos(numIKJoints);
  
  // input values are current Euler angles
  // output values are current handle positions
  double input_x_values[FKInputDim];
  double output_y_values[FKOutputDim];

  for(int i=0; i<numJoints; i++)
  {
    double temptArray[3] = {0.0,0.0,0.0};
    jointEulerAngles[i].convertToArray(temptArray);
    input_x_values[3*i+0] = temptArray[0];
    input_x_values[3*i+1] = temptArray[1];
    input_x_values[3*i+2] = temptArray[2];
    memset(&temptArray, 0, sizeof(double[3]));
  }

  // call the function to get the output values(current handle position)
  ::function(adolc_tagID, FKOutputDim, FKInputDim, input_x_values, output_y_values);

  // n----->input
  // m----->output
  double jacobianMatrix[m*n];
  double * jacobianMatrixEachRow[m];

  // pointer array where each pointer points to one row of the jacobian matrix
  for(int i=0; i<m; i++)
    jacobianMatrixEachRow[i] = &jacobianMatrix[i*n];

  // each row is the gradient of one output component of the function
  ::jacobian(adolc_tagID, m, n, input_x_values,jacobianMatrixEachRow); 

  // now we solve A x = b using Eigen
  // here A = (Jt*J + alpha*I)
  // here b = Jt * (change of handle global positions: deta pos)
  // theta (x) is n*1 vector representing the change of Euler angles we want to find. 

  // get the n*n identical matrix 
  Eigen::MatrixXd I(n, n);
  I = Eigen::MatrixXd::Identity(n, n);
  
  // define a m * n Eigen column-major matrix---> J matrix
  Eigen::MatrixXd J(m,n); 
  for(int rowID = 0; rowID < m; rowID++)
    for(int colID = 0; colID < n; colID++)    
      J(rowID,colID) = jacobianMatrix[n*rowID+colID];

  // get the transposed matrix
  Eigen::MatrixXd Jt(n,m);
  Jt = J;
  Jt.transposeInPlace();

  // get the left side matrix A
  Eigen::MatrixXd A(n, n);
  A = Jt * J + alpha * I;

  vector<Vec3d> getCurrentPos(numIKJoints);

  // convert double to vec3d
  for(int i=0 ;i<numIKJoints; i++)
  {
    double temptArray[3] = {0.0,0.0,0.0};
    temptArray[0] = output_y_values[3*i+0];
    temptArray[1] = output_y_values[3*i+1];
    temptArray[2] = output_y_values[3*i+2];
    getCurrentPos[i] = Vec3d(temptArray);
    memset(&temptArray, 0, sizeof(double[3]));
  }

  // get the vectors, which are the change of handle positions
  for(int i=0; i<numIKJoints; i++)
    deltaPos[i] = targetHandlePositions[i]-getCurrentPos[i];
 
  // convert deltaPos to double array
  vector<double> getDeltaPos(m);
  for(int i=0; i<numIKJoints; i++)
  {
    double temptArray[3] = {0.0,0.0,0.0};
    deltaPos[i].convertToArray(temptArray);
    getDeltaPos[3*i+0] = temptArray[0];
    getDeltaPos[3*i+1] = temptArray[1];
    getDeltaPos[3*i+2] = temptArray[2];
    memset(&temptArray, 0, sizeof(double[3]));
  }

  // get the right side vector: b
  // here b = Jt * (change of handle global positions: deta pos)
  Eigen::VectorXd deltaPosVector(m); 
  for(int i=0; i<m; i++)
    deltaPosVector[i] = getDeltaPos[i];
 
  Eigen::VectorXd b(m); 
  b = Jt * deltaPosVector;

  // now solve for x in A x = b
  // get the change of Euler angle x
  Eigen::VectorXd x = A.ldlt().solve(b);

  // convert VectorXd to vector<double>
  vector<double> deltaEuler(n);
  for(int i=0; i<n; i++)
    deltaEuler[i] = x[i];

  // convert vector<double> to Euler vec3d
  vector<Vec3d> getDeltaEuler(numJoints);
  for(int i=0; i<numJoints; i++)
  {
    double temptArray[3] = {0.0,0.0,0.0};
    temptArray[0] = deltaEuler[3*i+0];
    temptArray[1] = deltaEuler[3*i+1];
    temptArray[2] = deltaEuler[3*i+2];
    getDeltaEuler[i] = Vec3d(temptArray);
    memset(&temptArray, 0, sizeof(double[3]));
  }

  // upodate the Euler angles
  for(int i=0; i<numJoints; i++)
    jointEulerAngles[i] += getDeltaEuler[i];
}

