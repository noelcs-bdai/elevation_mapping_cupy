//
// Created by rgrandia on 16.03.22.
//

#pragma once

#include <string>

#include <grid_map_core/GridMap.hpp>
#include <Eigen/Dense>

namespace convex_plane_decomposition {
/** Scalar type. */
using scalar_t = double;
/** Dynamic-size matrix type. */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

/**
 * Load an elevation map from a grey scale image.
 *
 * @param filePath : absolute path to file
 * @param elevationLayer : name of the elevation layer
 * @param frameId : frame assigned to loaded map
 * @param resolution : map resolution [m/pixel]
 * @param scale : distance [m] between lowest and highest point in the map
 * @return Gridmap with the loaded image as elevation layer.
 */
grid_map::GridMap loadGridmapFromHeightmap(const std::string& filePath, const std::string& elevationLayer, const std::string& frameId,
                                       double resolution, double scale, double x_pos = 0.0);

}  // namespace convex_plane_decomposition