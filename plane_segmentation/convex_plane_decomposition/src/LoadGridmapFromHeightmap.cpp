//
// Created by rgrandia on 16.03.22.
//

#include "convex_plane_decomposition/LoadGridmapFromHeightmap.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <fstream>

namespace convex_plane_decomposition {

// https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

static bool addLayerFromHeightmap(const matrix_t& heightmap, const std::string& layer,
                              grid_map::GridMap& gridMap, const float lowerValue = 0.0,
                              const float upperValue = 1.0)
{
  if (gridMap.getSize()(0) != heightmap.rows() || gridMap.getSize()(1) != heightmap.cols()) {
    std::cerr << "Heightmap size does not correspond to grid map size!" << std::endl;
    return false;
  }

  const float mapValueDifference = upperValue - lowerValue;

  float maxHeightmapValue = heightmap.maxCoeff();

  gridMap.add(layer);
  grid_map::Matrix& data = gridMap[layer];

  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    const grid_map::Index imageIndex = iterator.getUnwrappedIndex();

    // Compute value.
    const scalar_t heightmapValue = heightmap(imageIndex(0), imageIndex(1));
    // const float mapValue = lowerValue + mapValueDifference * ((float) heightmapValue / maxHeightmapValue);
    data(gridMapIndex(0), gridMapIndex(1)) = heightmapValue;
  }

  return true;
}

grid_map::GridMap loadGridmapFromHeightmap(const std::string& filePath, const std::string& elevationLayer, const std::string& frameId,
                                       double resolution, double scale, double x_pos) {
  // Read the file
  matrix_t heightmap = load_csv<matrix_t>(filePath);

  // Check for invalid input
  if (heightmap.size() == 0) {
    throw std::runtime_error("Could not open or find the heightmap");
  }

  // Min max values
  // double minValue, maxValue;
  // cv::minMaxLoc(image, &minValue, &maxValue);

  grid_map::GridMap mapOut({elevationLayer});
  const grid_map::Position position = grid_map::Position(x_pos, 0.0);
  mapOut.setFrameId(frameId);
  const double lengthX = resolution * heightmap.rows();
  const double lengthY = resolution * heightmap.cols();
  grid_map::Length length(lengthX, lengthY);
  mapOut.setGeometry(length, resolution, position);
  addLayerFromHeightmap(heightmap, elevationLayer, mapOut, float(0.0), float(scale));
  // grid_map::GridMapCvConverter::initializeFromImage(image, resolution, mapOut, grid_map::Position(0.0, 0.0));
  // grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, elevationLayer, mapOut, float(0.0), float(scale), 0.5);
  return mapOut;
}

}  // namespace convex_plane_decomposition