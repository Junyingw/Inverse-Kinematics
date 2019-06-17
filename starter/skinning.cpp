#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry> 
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Spring 2019
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }
    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

/************************************** To implement LBS, uncomment this part ************************************/
// // Linear Blending Skinning (LBS) 
// void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions, FK * fk) const
// {
//   // Students should implement this
//   // jointSkinTransforms is an array of transformations, one per joint. For each joint, we have: 
//   // jointSkinTransform = globalTransform * globalRestTransform^{-1}

//   // newVertexPosition = totalSum(jointWeight * jointTransformMatrix * restPosition_v)
//   vector<Vec4d> getRestMeshVertexVec4d(numMeshVertices); 
//   vector<Vec4d> getNewMeshVertexVec4d(numMeshVertices); 
 
//   // get the joint skin transform matirx 
//   for(int i=0; i<numMeshVertices; i++)
//      getRestMeshVertexVec4d[i] = Vec4d(restMeshVertexPositions[3*i+0],restMeshVertexPositions[3*i+1],restMeshVertexPositions[3*i+2],1.0);
  
//   // intialize getNewMeshVertexVec4d
//   for(int i=0; i<numMeshVertices; i++)
//   {
//     Vec4d value = Vec4d(0.0,0.0,0.0,0.0);
//     getNewMeshVertexVec4d[i] = value;
//   }

//   // get the new mesh vertex positions
//   for(int i=0; i<numMeshVertices; i++)
//     for(size_t j=0; j<numJointsInfluencingEachVertex; j++)
//       getNewMeshVertexVec4d[i] += meshSkinningWeights[i * numJointsInfluencingEachVertex + j]*
//                       jointSkinTransforms[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]]*
//                       getRestMeshVertexVec4d[i];
   
//   // convert vertex to double array
//   for(int i=0; i<numMeshVertices; i++)
//   {
//     double temptArray[4];
//     getNewMeshVertexVec4d[i].convertToArray(temptArray);
//     newMeshVertexPositions[3 * i + 0]= temptArray[0];
//     newMeshVertexPositions[3 * i + 1]= temptArray[1];
//     newMeshVertexPositions[3 * i + 2]= temptArray[2]; 
//     memset(&temptArray, 0, sizeof(double[4]));
//   }
// }

/************************************** To implement DQS, uncomment this part ************************************/
// Dual Quaternions Skinning (DQS)
void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions, FK* fk) const
{

  // std::string sep = "\n----------------------------------------\n";
  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  int num = fk->getNumJoints(); 

  vector<Mat3d> getTransformMat3d(num);
  vector<Eigen::Matrix3d> getTransformMat(num); 
  vector<Eigen::Quaterniond> getJointQuaternion(num);
  vector<Mat4d> getJointTranslationMat4d(num);
  vector<Vec3d> getJointTranslationVec(num);

  vector<Vec3d> getNewTranslationVec(numMeshVertices);

  vector<Eigen::Quaterniond> getNewQuaternion(numMeshVertices);
  vector<Eigen::Matrix3d> getNewRotationMat3d(numMeshVertices);

  vector<Mat3d> getNewRotationMat(numMeshVertices);

  vector<Vec4d> getRestVertexMeshVec4d(numMeshVertices); 
  vector<Vec4d> getNewVertexMeshVec4d(numMeshVertices); 

  vector<RigidTransform4d> getNewTransformMat(numMeshVertices);

  // get translation vectors
  for(int i=0; i<num; i++)
  {
    getJointTranslationMat4d[i] = Mat4d(jointSkinTransforms[i]);
    //getJointTranslationMat4d[i].print();
    double temptArray[16];
    getJointTranslationMat4d[i].convertToArray(temptArray);
    getJointTranslationVec[i] = Vec3d(temptArray[3],temptArray[7],temptArray[11]);
    memset(&temptArray, 0, sizeof(double[16]));
  }

  // convert rigid joints transformation matrices to quaternions
  for(int i=0; i<num; i++)
  {
    double temptArray[9];
    getTransformMat3d[i] = jointSkinTransforms[i].getLinearTrans();
    getTransformMat3d[i].convertToArray(temptArray);

    // define a 3x3 Eigen temporary  matrix
    Eigen::MatrixXd temptMat(3,3); 

    // assign values to temporary  matrix
    for(int rowID = 0; rowID < 3; rowID++)
      for(int colID = 0; colID < 3; colID++)
        temptMat(rowID,colID) = temptArray[3*rowID+colID];

    getTransformMat[i] = Eigen::Matrix3d(temptMat);
    //getJointQuaternion[i] = Eigen::Quaterniond(getTransformMat[i]);
    getJointQuaternion[i] = getTransformMat[i];
    getJointQuaternion[i].normalized();
  }
  
  // intialize 
  for(int i=0; i<numMeshVertices; i++)
  {
    // quaternions
    Eigen::Quaterniond value0 = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
    getNewQuaternion[i] = value0;
    
    // translation vectors
    Vec3d value1 = Vec3d(0.0,0.0,0.0);
    getNewTranslationVec[i] = value1;

  }
  
  // get the new quaternions for each vertex.
  for(int i=0; i<numMeshVertices; i++)
  {
     for(int j=0; j<numJointsInfluencingEachVertex; j++)
    {
      // get new quaternions
      getNewQuaternion[i].coeffs() += getJointQuaternion[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]].coeffs()*
                                  meshSkinningWeights[i * numJointsInfluencingEachVertex + j];

      // get new translation vector
      getNewTranslationVec[i] += meshSkinningWeights[i * numJointsInfluencingEachVertex + j] *
                                  getJointTranslationVec[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]];

    }
    getNewQuaternion[i].normalized();
  }

  // convert quaternions to the new rotation matrices 
  for(int i=0; i<numMeshVertices; i++)
  {
    getNewRotationMat3d[i] = getNewQuaternion[i].toRotationMatrix();

    // convert data type
    double temptArray[9];
    for(int rowID = 0; rowID < 3; rowID++)
      for(int colID = 0; colID < 3; colID++)
        temptArray[3*rowID+colID] = getNewRotationMat3d[i](rowID,colID); 

    getNewRotationMat[i] = Mat3d(temptArray);
    memset(&temptArray, 0, sizeof(double[9]));
 
    // combine rotation matrices and translation vectors together
    getNewTransformMat[i] = AffineTransform4d(getNewRotationMat[i],getNewTranslationVec[i]);
  }
    
  // get the joint skin transform matirx 
  for(int i=0; i<numMeshVertices; i++)
  {
    getRestVertexMeshVec4d[i] = Vec4d(restMeshVertexPositions[3*i+0],restMeshVertexPositions[3*i+1],restMeshVertexPositions[3*i+2], 1.0);
    getNewVertexMeshVec4d[i] = getNewTransformMat[i] * getRestVertexMeshVec4d[i];

  }
  
  // convert vertex to double array
  for(int i=0; i<numMeshVertices; i++)
  {
    double temptArray[4];
    getNewVertexMeshVec4d[i].convertToArray(temptArray);
    newMeshVertexPositions[3 * i + 0]= temptArray[0];
    newMeshVertexPositions[3 * i + 1]= temptArray[1];
    newMeshVertexPositions[3 * i + 2]= temptArray[2]; 

    memset(&temptArray, 0, sizeof(double[3]));
  }
}
