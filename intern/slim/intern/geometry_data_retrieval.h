/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <Eigen/Dense>
#include <stdio.h>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include "uv_initializer.h"

using namespace Eigen;
using namespace igl;

namespace retrieval {

struct GeometryData {
  Map<MatrixXd> vertexPositions3D = Map<MatrixXd>(NULL, 0, 0);
  Map<MatrixXd> uvPositions2D = Map<MatrixXd>(NULL, 0, 0);
  MatrixXd positionsOfPinnedVertices2D;
  Map<Matrix<double, Dynamic, Dynamic, RowMajor>> positionsOfExplicitlyPinnedVertices2D =
      Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(NULL, 0, 0);

  Map<MatrixXi> facesByVertexindices = Map<MatrixXi>(NULL, 0, 0);
  Map<MatrixXi> edgesByVertexindices = Map<MatrixXi>(NULL, 0, 0);
  VectorXi PinnedVertexIndices;
  Map<VectorXi> ExplicitlyPinnedVertexIndices = Map<VectorXi>(NULL, 0);

  Map<VectorXd> edgeLengths = Map<VectorXd>(NULL, 0);
  Map<VectorXi> boundaryVertexIndices = Map<VectorXi>(NULL, 0);
  Map<VectorXf> weightsPerVertex = Map<VectorXf>(NULL, 0);

  int COLUMNS_2 = 2;
  int COLUMNS_3 = 3;
  int numberOfVertices;
  int numberOfFaces;
  int numberOfEdgesTwice;
  int numberOfBoundaryVertices;
  int numberOfPinnedVertices;

  bool withWeightedParameteriztion;
  double weightInfluence;
};

void constructSlimData(GeometryData &gd,
                       SLIMData *slimData,
                       bool skipInitialization,
                       int reflection_mode,
                       double relativeScale);

void retrievePinnedVertices(GeometryData &gd, bool borderVerticesArePinned);

void retrieveGeometryDataMatrices(const SLIMMatrixTransfer *transferredData,
                                  const int uvChartIndex,
                                  GeometryData &gd);
}  // namespace retrieval
