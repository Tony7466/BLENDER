/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <stdio.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

enum Method { TUTTE, HARMONIC, MVC };

namespace slim {

void convex_border_parameterization(const Eigen::MatrixXi &F,
                                    const Eigen::MatrixXd &V,
                                    const Eigen::MatrixXi &E,
                                    const Eigen::VectorXd &EL,
                                    const Eigen::VectorXi &bnd,
                                    const Eigen::MatrixXd &bnd_uv,
                                    Eigen::MatrixXd &UV,
                                    Method method);

void mvc(const Eigen::MatrixXi &F,
         const Eigen::MatrixXd &V,
         const Eigen::MatrixXi &E,
         const Eigen::VectorXd &EL,
         const Eigen::VectorXi &bnd,
         const Eigen::MatrixXd &bnd_uv,
         Eigen::MatrixXd &UV);

void harmonic(const Eigen::MatrixXi &F,
              const Eigen::MatrixXd &V,
              const Eigen::MatrixXi &E,
              const Eigen::VectorXd &EL,
              const Eigen::VectorXi &bnd,
              const Eigen::MatrixXd &bnd_uv,
              Eigen::MatrixXd &UV);

void tutte(const Eigen::MatrixXi &F,
           const Eigen::MatrixXd &V,
           const Eigen::MatrixXi &E,
           const Eigen::VectorXd &EL,
           const Eigen::VectorXi &bnd,
           const Eigen::MatrixXd &bnd_uv,
           Eigen::MatrixXd &UV);

void harmonic(const Eigen::MatrixXd &V,
              const Eigen::MatrixXi &F,
              const Eigen::MatrixXi &B,
              const Eigen::MatrixXd &bnd_uv,
              int powerOfHarmonicOperaton,
              Eigen::MatrixXd &UV);

void mapVerticesToConvexBorder(Eigen::MatrixXd &vertex_positions);

int count_flips(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXd &uv);

}
