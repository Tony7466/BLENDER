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

    void transfer_uvs_back_to_native_part_live(SLIMMatrixTransfer* mt, Eigen::MatrixXd& uv, int uv_chart_index)
    {
        if (!mt->succeeded[uv_chart_index]) {
            return;
        }

        double* uv_coordinate_array = mt->uv_matrices[uv_chart_index];
        int number_of_vertices = mt->n_verts[uv_chart_index];

        for (int i = 0; i < number_of_vertices; i++) {
            *(uv_coordinate_array++) = uv(i, 0);
            *(uv_coordinate_array++) = uv(i, 1);
        }
    }

    void transfer_uvs_back_to_native_part(SLIMMatrixTransfer* mt, Eigen::MatrixXd& uv, int uv_chart_index)
    {
        if (!mt->succeeded[uv_chart_index]) {
            return;
        }

        double* uv_coordinate_array;
        uv_coordinate_array = mt->uv_matrices[uv_chart_index];
        int number_of_vertices = mt->n_verts[uv_chart_index];

        for (int i = 0; i < number_of_vertices; i++) {
            for (int j = 0; j < 2; j++) {
                uv_coordinate_array[i * 2 + j] = uv(i, j);
            }
        }
    }

    Eigen::MatrixXd get_interactive_result_blended_with_original(float blend, const SLIMData* slim_data)
    {
        Eigen::MatrixXd original_map_weighted = blend * slim_data->oldUVs;
        Eigen::MatrixXd interactive_result_map = (1.0 - blend) * slim_data->V_o;
        return original_map_weighted + interactive_result_map;
    }

    /*
      Executes a single iteration of SLIM, must follow a proper setup & initialisation.
     */
    void param_slim_single_iteration(SLIMMatrixTransfer* mt, int uv_chart_index, SLIMData* slim_data)
    {
        int number_of_iterations = 1;
        try_slim_solve(mt, uv_chart_index, *slim_data, number_of_iterations);
    }

    static void adjust_pins(
        SLIMData* slim_data,
        int n_pins,
        int* pinned_vertex_indices,
        double* pinned_vertex_positions2d,
        int n_selected_pins,
        int* selected_pins)
    {
        if (!slim_data->valid) {
            return;
        }

        Eigen::VectorXi old_pin_indices = slim_data->b;
        Eigen::MatrixXd old_pin_positions = slim_data->bc;

        slim_data->b.resize(n_pins);
        slim_data->bc.resize(n_pins, 2);

        int old_pin_pointer = 0;
        int new_pin_pointer = 0;
        int selected_pin_pointer = 0;

        while (new_pin_pointer < n_pins) {

            int pinned_vertex_index = pinned_vertex_indices[new_pin_pointer];
            slim_data->b(new_pin_pointer) = pinned_vertex_index;

            while (old_pin_indices(old_pin_pointer) < pinned_vertex_index) {
                ++old_pin_pointer;
                if (old_pin_pointer == old_pin_indices.size()) {
                    break;
                }
            }

            while (selected_pins[selected_pin_pointer] < pinned_vertex_index) {
                ++selected_pin_pointer;
                if (selected_pin_pointer == n_selected_pins) {
                    break;
                }
            }

            if (!(pinned_vertex_index == selected_pins[selected_pin_pointer]) &&
                old_pin_indices(old_pin_pointer) == pinned_vertex_index) {
                slim_data->bc.row(new_pin_pointer) = old_pin_positions.row(old_pin_pointer);
            }
            else {
                slim_data->bc(new_pin_pointer, 0) = pinned_vertex_positions2d[2 * new_pin_pointer];
                slim_data->bc(new_pin_pointer, 1) = pinned_vertex_positions2d[2 * new_pin_pointer + 1];
            }

            ++new_pin_pointer;
        }
    }

    /*
      Executes several iterations of SLIM when used with LiveUnwrap
     */

    void param_slim_live_unwrap(SLIMMatrixTransfer* mt,
        int uv_chart_index,
        SLIMData* slim_data,
        int n_pins,
        int* pinned_vertex_indices,
        double* pinned_vertex_positions2d,
        int n_selected_pins,
        int* selected_pins)
    {
        int number_of_iterations = 3;
        adjust_pins(slim_data,
            n_pins,
            pinned_vertex_indices,
            pinned_vertex_positions2d,
            n_selected_pins,
            selected_pins);
        // recompute current energy
        // recompute_energy(*slim_data);
        try_slim_solve(mt, uv_chart_index, *slim_data, number_of_iterations);
    }

    void param_slim(SLIMMatrixTransfer* mt,
        int n_iterations,
        bool border_vertices_are_pinned,
        bool skip_initialization)
    {

        igl::Timer timer;
        timer.start();

        for (int uv_chart_index = 0; uv_chart_index < mt->n_charts; uv_chart_index++) {
            SLIMData* slim_data = setup_slim(
                mt, n_iterations, uv_chart_index, timer, border_vertices_are_pinned, skip_initialization);
            try_slim_solve(mt, uv_chart_index, *slim_data, n_iterations);

            correct_map_surface_area_if_necessary(slim_data);
            transfer_uvs_back_to_native_part(mt, slim_data->V_o, uv_chart_index);

            free_slim_data(slim_data);
        }
    };

    void initialize_uvs(GeometryData& gd, SLIMData* slim_data)
    {

        MatrixXd vertex_positions2d = slim_data->V;
        MatrixXi faces_by_vertex_indices = slim_data->F;
        VectorXi boundary_vertex_indices = gd.boundary_vertex_indices;
        MatrixXd uv_positions2d = slim_data->V_o;

        Eigen::MatrixXd uv_positions_of_boundary(boundary_vertex_indices.rows(), 2);
        map_vertices_to_convex_border(uv_positions_of_boundary);

        bool all_vertices_on_boundary = (slim_data->V_o.rows() == uv_positions_of_boundary.rows());
        if (all_vertices_on_boundary) {
            slim_data->V_o = uv_positions_of_boundary;
            return;
        }

        mvc(gd.faces_by_vertexindices,
            gd.vertex_positions3d,
            gd.edges_by_vertexindices,
            gd.edge_lengths,
            boundary_vertex_indices,
            uv_positions_of_boundary,
            slim_data->V_o);
    }

    void initialize_if_needed(GeometryData& gd, SLIMData* slim_data)
    {
        BLI_assert(slim_data->valid);

        if (!slim_data->skipInitialization) {
            initialize_uvs(gd, slim_data);
        }
    }

    /*
      Transfers all the matrices from the native part and initialises slim.
     */
    SLIMData* setup_slim(const SLIMMatrixTransfer* transferred_data,
        int n_iterations,
        int uv_chart_index,
        igl::Timer& timer,
        bool border_vertices_are_pinned,
        bool skip_initialization)
    {
        SLIMData* slim_data = new SLIMData();

        try {
            if (!transferred_data->succeeded[uv_chart_index]) {
                throw SlimFailedException();
            }

            GeometryData geometry_data;
            retrieve_geometry_data_matrices(transferred_data, uv_chart_index, geometry_data);

            retrieve_pinned_vertices(geometry_data, border_vertices_are_pinned);
            transferred_data->n_pinned_vertices[uv_chart_index] = geometry_data.number_of_pinned_vertices;

            construct_slim_data(geometry_data,
                slim_data,
                skip_initialization,
                transferred_data->reflection_mode,
                transferred_data->relative_scale);
            slim_data->nIterations = n_iterations;

            initialize_if_needed(geometry_data, slim_data);
            transform_initialization_if_necessary(*slim_data);

            correct_mesh_surface_area_if_necessary(slim_data);

            slim_precompute(slim_data->V,
                slim_data->F,
                slim_data->V_o,
                *slim_data,
                slim_data->slim_energy,
                slim_data->b,
                slim_data->bc,
                slim_data->soft_const_p);
        }
        catch (SlimFailedException&) {
            slim_data->valid = false;
            transferred_data->succeeded[uv_chart_index] = false;
        }

        return slim_data;
    }

    void try_slim_solve(SLIMMatrixTransfer* mt, int uv_chart_index, SLIMData& data, int iter_num)
    {
        if (!mt->succeeded[uv_chart_index]) {
            return;
        }

        try {
            slim_solve(data, iter_num);
        }
        catch (SlimFailedException&) {
            mt->succeeded[uv_chart_index] = false;
        }
    }

    void free_slim_data(SLIMData* slim_data)
    {
        delete slim_data;
    };

}
