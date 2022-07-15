/* SPDX-License-Identifier: GPL-2.0-or-later */


#include "least_squares_relocator.h"

#include "slim.h"

#include <Eigen/Dense>
#include <iostream>

#include "BLI_assert.h"


namespace slim {

    using namespace Eigen;
    using namespace igl;

void applyTransformation(SLIMData &slimData, Matrix2d &transformationMatrix)
{
  BLI_assert(slimData.valid);

  for (int i = 0; i < slimData.V_o.rows(); i++) {
    slimData.V_o.row(i) = transformationMatrix * slimData.V_o.row(i).transpose();
  }
}

void applyTranslation(SLIMData &slimData, Vector2d &translationVector)
{
  BLI_assert(slimData.valid);

  for (int i = 0; i < slimData.V_o.rows(); i++) {
    slimData.V_o.row(i) = translationVector.transpose() + slimData.V_o.row(i);
  }
}

void retrievePositionsOfPinnedVerticesInInitialization(
    const MatrixXd &allUVPositionsInInitialization,
    const VectorXi &indicesOfPinnedVertices,
    MatrixXd &positionOfPinnedVerticesInInitialization)
{
  int i = 0;
  for (VectorXi::InnerIterator it(indicesOfPinnedVertices, 0); it; ++it, i++) {
    int vertexIndex = it.value();
    positionOfPinnedVerticesInInitialization.row(i) = allUVPositionsInInitialization.row(
        vertexIndex);
  }
}

void flipInputGeometry(SLIMData &slimData)
{
  BLI_assert(slimData.valid);
  // slimData.V.col(0) *= -1;

  VectorXi temp = slimData.F.col(0);
  slimData.F.col(0) = slimData.F.col(2);
  slimData.F.col(2) = temp;
}

void computeCentroid(const MatrixXd &pointCloud, Vector2d &centroid)
{
  centroid << pointCloud.col(0).sum(), pointCloud.col(1).sum();
  centroid /= pointCloud.rows();
};

/*
 Finds scaling matrix

 T = |a 0|
     |0 a|


 s.t. if to each point p in the inizialized map the following is applied

 T*p

 we get the closest scaling of the positions of the vertices in the initialized map to the pinned
 vertices in a least squares sense. We find them by solving

 argmin_{t}	At = p

 i.e.:

 | x_1 |           |u_1|
 |  .  |           | . |
 |  .  |           | . |
 | x_n |           |u_n|
 | y_1 | * | a | = |v_1|
 |  .  |           | . |
 |  .  |           | . |
 | y_n |           |v_n|

 t is of dimension 1 x 1 and p of dimension 2*numberOfPinnedVertices x 1
 is the vector holding the uv positions of the pinned vertices.
 */
void computeLeastSquaresScaling(MatrixXd centeredPins,
                                MatrixXd centeredInitializedPins,
                                Matrix2d &transformationMatrix)
{
  int numberOfPinnedVertices = centeredPins.rows();

  MatrixXd A = MatrixXd::Zero(numberOfPinnedVertices * 2, 1);
  A << centeredInitializedPins.col(0), centeredInitializedPins.col(1);

  VectorXd p(2 * numberOfPinnedVertices);
  p << centeredPins.col(0), centeredPins.col(1);

  VectorXd t = A.colPivHouseholderQr().solve(p);
  t(0) = abs(t(0));
  transformationMatrix << t(0), 0, 0, t(0);
}

void computLeastSquaresRotationScaleOnly(SLIMData &slimData,
                                         Vector2d &translationVector,
                                         Matrix2d &transformationMatrix,
                                         bool isFlipAllowed)
{
  BLI_assert(slimData.valid);

  MatrixXd positionOfInitializedPins(slimData.b.rows(), 2);
  retrievePositionsOfPinnedVerticesInInitialization(
      slimData.V_o, slimData.b, positionOfInitializedPins);

  Vector2d centroidOfInitialized;
  computeCentroid(positionOfInitializedPins, centroidOfInitialized);

  Vector2d centroidOfPins;
  computeCentroid(slimData.bc, centroidOfPins);

  MatrixXd centeredInitializedPins = positionOfInitializedPins.rowwise().operator-(
      centroidOfInitialized.transpose());
  MatrixXd centeredpins = slimData.bc.rowwise().operator-(centroidOfPins.transpose());

  MatrixXd S = centeredInitializedPins.transpose() * centeredpins;

  JacobiSVD<MatrixXd> svd(S, ComputeFullU | ComputeFullV);

  Matrix2d VU_T = svd.matrixV() * svd.matrixU().transpose();

  Matrix2d singularValues = Matrix2d::Identity();

  bool containsReflection = VU_T.determinant() < 0;
  if (containsReflection) {
    if (!isFlipAllowed) {
      singularValues(1, 1) = VU_T.determinant();
    }
    else {
      flipInputGeometry(slimData);
    }
  }

  computeLeastSquaresScaling(centeredpins, centeredInitializedPins, transformationMatrix);

  transformationMatrix = transformationMatrix * svd.matrixV() * singularValues *
                         svd.matrixU().transpose();

  translationVector = centroidOfPins - transformationMatrix * centroidOfInitialized;
}

void computeTransformationMatrix2Pins(const SLIMData &slimData, Matrix2d &transformationMatrix)
{
  BLI_assert(slimData.valid);

  Vector2d pinnedPositionDifferenceVector = slimData.bc.row(0) - slimData.bc.row(1);
  Vector2d initializedPositionDifferenceVector = slimData.V_o.row(slimData.b(0)) -
                                                 slimData.V_o.row(slimData.b(1));

  double scale = pinnedPositionDifferenceVector.norm() /
                 initializedPositionDifferenceVector.norm();

  pinnedPositionDifferenceVector.normalize();
  initializedPositionDifferenceVector.normalize();

  // TODO: sometimes rotates in wrong direction
  double cosAngle = pinnedPositionDifferenceVector.dot(initializedPositionDifferenceVector);
  double sinAngle = sqrt(1 - pow(cosAngle, 2));

  transformationMatrix << cosAngle, -sinAngle, sinAngle, cosAngle;
  transformationMatrix = (Matrix2d::Identity() * scale) * transformationMatrix;
}

void computeTranslation1Pin(const SLIMData &slimData, Vector2d &translationVector)
{
  BLI_assert(slimData.valid);
  translationVector = slimData.bc.row(0) - slimData.V_o.row(slimData.b(0));
}

void transformInitializedMap(SLIMData &slimData)
{
  BLI_assert(slimData.valid);
  Matrix2d transformationMatrix;
  Vector2d translationVector;

  int numberOfPinnedVertices = slimData.b.rows();

  switch (numberOfPinnedVertices) {
    case 0:
      std::cout << "No transformation possible because no pinned vertices exist." << std::endl;
      return;
    case 1:  // only translation is needed with one pin
      computeTranslation1Pin(slimData, translationVector);
      applyTranslation(slimData, translationVector);
      break;
    case 2:
      computeTransformationMatrix2Pins(slimData, transformationMatrix);
      applyTransformation(slimData, transformationMatrix);
      computeTranslation1Pin(slimData, translationVector);
      applyTranslation(slimData, translationVector);
      break;
    default:

      bool flipAllowed = slimData.reflection_mode == 0;

      computLeastSquaresRotationScaleOnly(
          slimData, translationVector, transformationMatrix, flipAllowed);

      applyTransformation(slimData, transformationMatrix);
      applyTranslation(slimData, translationVector);

      break;
  }
}

bool isTranslationNeeded(const SLIMData &slimData)
{
  BLI_assert(slimData.valid);
  bool pinnedVerticesExist = (slimData.b.rows() > 0);
  bool wasInitialized = !slimData.skipInitialization;
  return wasInitialized && pinnedVerticesExist;
}

void transformInitializationIfNecessary(SLIMData &slimData)
{
  BLI_assert(slimData.valid);

  if (!isTranslationNeeded(slimData)) {
    return;
  }

  transformInitializedMap(slimData);
}
}
