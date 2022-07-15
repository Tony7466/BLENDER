/* SPDX-License-Identifier: GPL-2.0-or-later */


#include <iostream>
#include <stdlib.h>

#include "area_compensation.h"
#include "geometry_data_retrieval.h"
#include "least_squares_relocator.h"
#include "slim.h"
#include "uv_initializer.h"

#include "BLI_assert.h"

#include "igl/Timer.h"

#include "doublearea.h"
#include "igl/map_vertices_to_circle.h"

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "slim_parametrizer.h"


namespace slim {

    using namespace igl;
    using namespace Eigen;

    void transferUvsBackToNativePartLive(SLIMMatrixTransfer* mt, Eigen::MatrixXd& UV, int uvChartIndex)
    {
        if (!mt->succeeded[uvChartIndex]) {
            return;
        }

        double* uvCoordinateArray = mt->uv_matrices[uvChartIndex];
        int numberOfVertices = mt->n_verts[uvChartIndex];

        for (int i = 0; i < numberOfVertices; i++) {
            *(uvCoordinateArray++) = UV(i, 0);
            *(uvCoordinateArray++) = UV(i, 1);
        }
    }

    void transferUvsBackToNativePart(SLIMMatrixTransfer* mt, Eigen::MatrixXd& UV, int uvChartIndex)
    {
        if (!mt->succeeded[uvChartIndex]) {
            return;
        }

        double* uvCoordinateArray;
        uvCoordinateArray = mt->uv_matrices[uvChartIndex];
        int numberOfVertices = mt->n_verts[uvChartIndex];

        for (int i = 0; i < numberOfVertices; i++) {
            for (int j = 0; j < 2; j++) {
                uvCoordinateArray[i * 2 + j] = UV(i, j);
            }
        }
    }

    Eigen::MatrixXd getInteractiveResultBlendedWithOriginal(float blend, const SLIMData* slimData)
    {
        Eigen::MatrixXd originalMapWeighted = blend * slimData->oldUVs;
        Eigen::MatrixXd InteractiveResultMap = (1.0 - blend) * slimData->V_o;
        return originalMapWeighted + InteractiveResultMap;
    }

    /*
      Executes a single iteration of SLIM, must follow a proper setup & initialisation.
     */
    void param_slim_single_iteration(SLIMMatrixTransfer* mt, int uv_chart_index, SLIMData* slimData)
    {
        int numberOfIterations = 1;
        try_slim_solve(mt, uv_chart_index, *slimData, numberOfIterations);
    }

    static void adjustPins(SLIMData* slimData,
        int n_pins,
        int* pinnedVertexIndices,
        double* pinnedVertexPositions2D,
        int n_selected_pins,
        int* selected_pins)
    {
        if (!slimData->valid) {
            return;
        }

        Eigen::VectorXi oldPinIndices = slimData->b;
        Eigen::MatrixXd oldPinPositions = slimData->bc;

        slimData->b.resize(n_pins);
        slimData->bc.resize(n_pins, 2);

        int oldPinPointer = 0;
        int newPinPointer = 0;
        int selectedPinPointer = 0;

        while (newPinPointer < n_pins) {

            int pinnedVertexIndex = pinnedVertexIndices[newPinPointer];
            slimData->b(newPinPointer) = pinnedVertexIndex;

            while (oldPinIndices(oldPinPointer) < pinnedVertexIndex) {
                ++oldPinPointer;
                if (oldPinPointer == oldPinIndices.size()) {
                    break;
                }
            }

            while (selected_pins[selectedPinPointer] < pinnedVertexIndex) {
                ++selectedPinPointer;
                if (selectedPinPointer == n_selected_pins) {
                    break;
                }
            }

            if (!(pinnedVertexIndex == selected_pins[selectedPinPointer]) &&
                oldPinIndices(oldPinPointer) == pinnedVertexIndex) {
                slimData->bc.row(newPinPointer) = oldPinPositions.row(oldPinPointer);
            }
            else {
                slimData->bc(newPinPointer, 0) = pinnedVertexPositions2D[2 * newPinPointer];
                slimData->bc(newPinPointer, 1) = pinnedVertexPositions2D[2 * newPinPointer + 1];
            }

            ++newPinPointer;
        }
    }

    /*
      Executes several iterations of SLIM when used with LiveUnwrap
     */

    void param_slim_live_unwrap(SLIMMatrixTransfer* mt,
        int uv_chart_index,
        SLIMData* slimData,
        int n_pins,
        int* pinnedVertexIndices,
        double* pinnedVertexPositions2D,
        int n_selected_pins,
        int* selected_pins)
    {
        int numberOfIterations = 3;
        adjustPins(slimData,
            n_pins,
            pinnedVertexIndices,
            pinnedVertexPositions2D,
            n_selected_pins,
            selected_pins);
        // recompute current energy
        // recompute_energy(*slimData);
        try_slim_solve(mt, uv_chart_index, *slimData, numberOfIterations);
    }

    void param_slim(SLIMMatrixTransfer* mt,
        int nIterations,
        bool borderVerticesArePinned,
        bool skipInitialization)
    {

        igl::Timer timer;
        timer.start();

        for (int uvChartIndex = 0; uvChartIndex < mt->n_charts; uvChartIndex++) {
            SLIMData* slimData = setup_slim(
                mt, nIterations, uvChartIndex, timer, borderVerticesArePinned, skipInitialization);
            try_slim_solve(mt, uvChartIndex, *slimData, nIterations);

            correctMapSurfaceAreaIfNecessary(slimData);
            transferUvsBackToNativePart(mt, slimData->V_o, uvChartIndex);

            free_slim_data(slimData);
        }
    };

    void initializeUvs(GeometryData& gd, SLIMData* slimData)
    {

        MatrixXd vertexPositions2D = slimData->V;
        MatrixXi facesByVertexIndices = slimData->F;
        VectorXi boundaryVertexIndices = gd.boundaryVertexIndices;
        MatrixXd uvPositions2D = slimData->V_o;

        Eigen::MatrixXd uvPositionsOfBoundary(boundaryVertexIndices.rows(), 2);
        mapVerticesToConvexBorder(uvPositionsOfBoundary);

        bool allVerticesOnBoundary = (slimData->V_o.rows() == uvPositionsOfBoundary.rows());
        if (allVerticesOnBoundary) {
            slimData->V_o = uvPositionsOfBoundary;
            return;
        }

        mvc(gd.facesByVertexindices,
            gd.vertexPositions3D,
            gd.edgesByVertexindices,
            gd.edgeLengths,
            boundaryVertexIndices,
            uvPositionsOfBoundary,
            slimData->V_o);
    }

    void initializeIfNeeded(GeometryData& gd, SLIMData* slimData)
    {
        BLI_assert(slimData->valid);

        if (!slimData->skipInitialization) {
            initializeUvs(gd, slimData);
        }
    }

    /*
      Transfers all the matrices from the native part and initialises slim.
     */
    SLIMData* setup_slim(const SLIMMatrixTransfer* transferredData,
        int nIterations,
        int uvChartIndex,
        igl::Timer& timer,
        bool borderVerticesArePinned,
        bool skipInitialization)
    {
        SLIMData* slimData = new SLIMData();

        try {
            if (!transferredData->succeeded[uvChartIndex]) {
                throw SlimFailedException();
            }

            GeometryData geometryData;
            retrieveGeometryDataMatrices(transferredData, uvChartIndex, geometryData);

            retrievePinnedVertices(geometryData, borderVerticesArePinned);
            transferredData->n_pinned_vertices[uvChartIndex] = geometryData.numberOfPinnedVertices;

            constructSlimData(geometryData,
                slimData,
                skipInitialization,
                transferredData->reflection_mode,
                transferredData->relative_scale);
            slimData->nIterations = nIterations;

            initializeIfNeeded(geometryData, slimData);
            transformInitializationIfNecessary(*slimData);

            correctMeshSurfaceAreaIfNecessary(slimData);

            slim_precompute(slimData->V,
                slimData->F,
                slimData->V_o,
                *slimData,
                slimData->slim_energy,
                slimData->b,
                slimData->bc,
                slimData->soft_const_p);
        }
        catch (SlimFailedException&) {
            slimData->valid = false;
            transferredData->succeeded[uvChartIndex] = false;
        }

        return slimData;
    }

    void try_slim_solve(SLIMMatrixTransfer* mt, int uvChartIndex, SLIMData& data, int iter_num)
    {
        if (!mt->succeeded[uvChartIndex]) {
            return;
        }

        try {
            slim_solve(data, iter_num);
        }
        catch (SlimFailedException&) {
            mt->succeeded[uvChartIndex] = false;
        }
    }

    void free_slim_data(SLIMData* slimData)
    {
        delete slimData;
    };

}
