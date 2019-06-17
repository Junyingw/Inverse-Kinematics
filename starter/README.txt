
=============================== BUILD ON MAC OS 10.14.4 ================================

Unzip the files

> cd start
> make
> ./run.sh

Then you can see a completed armadillo with rendered skeleton. Use the handles to move it. 
Change the run.sh, then you can test other demos.

The default skinning algorithm is Linear Blending Skinning (LBS).

================================== FINISHED PARTS ======================================

* Skinning (skinning.cpp)

1) Finished the applySkinning() function
To implement it, use this equation:
newVertexPosition = TotalSum(jointWeight * jointTransformMatrix * restVertexPosition)

* Forward Kinematics (FK.cpp)

1) Finished the computeLocalAndGlobalTransforms() function
To implement it:
Firstly, get the local transform matrices. Use this equation:
M(Local) = M(JointOritation) * M(EulertoRotation) *  M(Translations)
Secondly, go recursively to get the globalTransform. Use this equation:
globalTransform = parentGlobalTransform * localTransform

2) Finished the computeSkinningTransforms() function
To implement it, use this equation:
skinTransform = globalTransform * invRestTransform

* Inverse Kinematics (IK.cpp)

1) Finished the forwardKinematicsFunction() function
Use the same logic as computeLocalAndGlobalTransforms()

2) Finished the train_adolc() function
To implement it:
Firstly, define adol_c inputs.
Secondly, use forwardKinematicsFunction() as the computed function f to get the outputs.

3) Finished the doIK() function
To implement it:
Solve A x = b using Eigen
here A = (Jt*J + alpha*I)
b = Jt * (change of handle global positions: delta pos)
theta (x) is n*1 vector representing the change of Euler angles we want to find. 

* Captured videos included (TestDemo_Video)

1) armadillo+dual quaternion skinning.mov
2) hand+linear blend skinning.mov

================================== EXTRA CREDITS ========================================

* Dual Quaternion Skinning (DQS)

To test this part, uncomment the DQS part in skinning.cpp, and comment LBS part.

To implement this:
1) Convert the jointSkinTransforms affine transformation matrixes into rotation matrices and translation vectors. Since I found that if I used Quaternion.h of Eigen, it could only convert 3*3 rotation matrix to quaternion, instead of 4*4 transformation matrix.
2) Use Eigen::Quaternion() convert rotation matrices to quaternions 
3) Compute normalized quaternions q using the related joints' weights. Compute translation vectors using the related joints' weights. 
4) Transform quaternion q back into rotation matrices. Combine rotation matrices with new translation vectors, using affine transform. 
5) Calculate the deformed skin vertex position, using the affine transformation matrices to multiply the rest position vectors.
 
Comparison between linear blend skinning and dual quaternion skinning:
1) DQS can avoid the loss of volume problem
2) Suppose we know two vertices positions: P1 and P2. If we use linear interpolation to get the new position, the new position will be lying on the segment between P1 and P2. If we use DQS, the new position will be lying on the arc circle contains P1 and P2, which will avoid mesh shrinkage.

====================================== ENJOY ===========================================

