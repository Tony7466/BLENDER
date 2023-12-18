/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#  include <openvdb/tools/Morphology.h>

namespace blender::nodes {

openvdb::tools::NearestNeighbors get_vdb_neighbors_mode(
    GeometryNodeGridNeighborTopology neighbors_mode);

}

#endif /* WITH_OPENVDB */
