#include <chrono>
#include <filesystem>
#include <iostream>
#include <unordered_set>
#include <vector>

#include <kiss_matcher/points/downsampling.hpp>
#include <kiss_matcher/points/point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
  // E.g.,
  // ./downsampling_speed_comparison
  // /media/shapelim/UX960NVMe1/kiss-matcher-viz/vbr-colosseo/target.pcd
  const std::string input_pcd_file = argv[1];

  // Load points
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  int src_load_result = pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *points_pcl);

  // --------------------------------------------------------------------------------
  // convert PCL points to kiss_matcher PointCloud
  auto start_conversion = std::chrono::high_resolution_clock::now();
  std::vector<Eigen::Vector3f> points_each;
  points_each.resize(points_pcl->size());
  for (size_t i = 0; i < points_pcl->size(); ++i) {
    points_each[i] =
        Eigen::Vector3f(points_pcl->points[i].x, points_pcl->points[i].y, points_pcl->points[i].z);
  }
  auto end_each_conversion = std::chrono::high_resolution_clock::now();
  auto conversion_time =
      std::chrono::duration<double, std::milli>(end_each_conversion - start_conversion).count();
  std::cout << "Time taken (raw): " << conversion_time << " ms" << std::endl;

  std::vector<Eigen::Vector3f> points_eigen;
  points_eigen.resize(points_pcl->size());
  for (size_t i = 0; i < points_pcl->size(); ++i) {
    points_eigen[i] = points_pcl->points[i].getVector3fMap();
  }
  auto end_eigen_conversion = std::chrono::high_resolution_clock::now();
  conversion_time =
      std::chrono::duration<double, std::milli>(end_eigen_conversion - end_each_conversion).count();
  std::cout << "Time taken (getVector3fMap): " << conversion_time << " ms" << std::endl;

  std::vector<Eigen::Vector3f> points_memcpy(points_pcl->size());
  std::memcpy(points_memcpy.data(),
              points_pcl->points.data(),
              points_pcl->size() * sizeof(Eigen::Vector3f));
  auto end_memcpy_conversion = std::chrono::high_resolution_clock::now();
  conversion_time =
      std::chrono::duration<double, std::milli>(end_memcpy_conversion - end_eigen_conversion)
          .count();
  std::cout << "Conversion time (Memcpy): " << conversion_time << " ms" << std::endl;

  // ToDo(hlim): Hmmm...this is much faster. Should we change the pipeline?
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>> points_eigen_map(
      reinterpret_cast<float*>(points_pcl->points.data()), 3, points_pcl->size());
  auto end_map_conversion = std::chrono::high_resolution_clock::now();
  conversion_time =
      std::chrono::duration<double, std::milli>(end_map_conversion - end_memcpy_conversion).count();
  std::cout << "Time taken (Eigen-map): " << conversion_time << " ms" << std::endl;
  std::cout << points_eigen_map.rows() << ", " << points_eigen_map.cols() << std::endl;

  std::vector<Eigen::Vector3f> points_map_then_assign(points_pcl->size());
  for (size_t i = 0; i < points_pcl->size(); ++i) {
    points_map_then_assign[i] = points_eigen_map.col(i);
  }
  auto end_map_to_vector_conversion = std::chrono::high_resolution_clock::now();
  conversion_time =
      std::chrono::duration<double, std::milli>(end_map_to_vector_conversion - end_map_conversion)
          .count();
  std::cout << "Conversion time (map then assign): " << conversion_time << " ms" << std::endl;

  std::vector<Eigen::Vector3f> points_fast(points_pcl->size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>>(
      reinterpret_cast<float*>(points_fast.data()), 3, points_pcl->size()) =
      Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::ColMajor>>(
          reinterpret_cast<float*>(points_pcl->points.data()), 3, points_pcl->size());

  auto end_direct_map_conversion = std::chrono::high_resolution_clock::now();
  conversion_time = std::chrono::duration<double, std::milli>(end_direct_map_conversion -
                                                              end_map_to_vector_conversion)
                        .count();
  std::cout << "Conversion time (Direct Eigen-map to std::vector): " << conversion_time << " ms"
            << std::endl;

  return 0;
}
