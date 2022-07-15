/* SPDX-License-Identifier: GPL-2.0-or-later */


#include "slim.h"

#include "BLI_assert.h"

#include <Eigen/Dense>
#include <doublearea.h>

using namespace Eigen;
using namespace igl;

namespace slim {

void correctGeometrySize(double surfaceAreaToMapAreaRatio,
                         MatrixXd &VertexPositions,
                         double desiredSurfaceAreaToMapRation)
{
  assert(surfaceAreaToMapAreaRatio > 0);
  double sqrtOfRatio = sqrt(surfaceAreaToMapAreaRatio / desiredSurfaceAreaToMapRation);
  VertexPositions = VertexPositions / sqrtOfRatio;
}

template<typename VertexPositionType, typename FaceIndicesType>
double computeSurfaceArea(const VertexPositionType V, const FaceIndicesType F)
{
  Eigen::VectorXd doubledAreaOfTriangles;
  igl::doublearea(V, F, doubledAreaOfTriangles);
  double areaOfMap = doubledAreaOfTriangles.sum() / 2;
  return areaOfMap;
}

void correctMapSurfaceAreaIfNecessary(SLIMData *slimData)
{
  if (!slimData->valid) {
    return;
  }

  bool meshSurfaceAreaWasCorrected = (slimData->expectedSurfaceAreaOfResultingMap != 0);
  int numberOfPinnedVertices = slimData->b.rows();
  bool noPinnedVerticesExist = numberOfPinnedVertices == 0;

  bool needsAreaCorrection = meshSurfaceAreaWasCorrected && noPinnedVerticesExist;
  if (!needsAreaCorrection) {
    return;
  }

  double areaOfresultingMap = computeSurfaceArea(slimData->V_o, slimData->F);
  if (!areaOfresultingMap) {
    return;
  }

  double resultingAreaToExpectedAreaRatio = areaOfresultingMap /
                                            slimData->expectedSurfaceAreaOfResultingMap;
  double desiredRatio = 1.0;
  correctGeometrySize(resultingAreaToExpectedAreaRatio, slimData->V_o, desiredRatio);
}

void correctMeshSurfaceAreaIfNecessary(SLIMData *slimData)
{
  BLI_assert(slimData->valid);

  int numberOfPinnedVertices = slimData->b.rows();
  bool pinnedVerticesExist = numberOfPinnedVertices > 0;
  bool needsAreaCorrection = slimData->skipInitialization || pinnedVerticesExist;

  if (!needsAreaCorrection) {
    return;
  }

  double areaOfPreinitializedMap = computeSurfaceArea(slimData->V_o, slimData->F);
  if (!areaOfPreinitializedMap) {
    return;
  }

  if (areaOfPreinitializedMap < 0) {
    areaOfPreinitializedMap *= -1;
  }

  slimData->expectedSurfaceAreaOfResultingMap = areaOfPreinitializedMap;
  double surfaceAreaOf3DMesh = computeSurfaceArea(slimData->V, slimData->F);
  double surfaceAreaToMapAreaRatio = surfaceAreaOf3DMesh / areaOfPreinitializedMap;

  double desiredRatio = 1.0;
  correctGeometrySize(surfaceAreaToMapAreaRatio, slimData->V, desiredRatio);
}
}
