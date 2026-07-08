// From https://github.com/koide3/small_gicp/blob/master/src/benchmark/downsampling_benchmark.cpp
#include <chrono>
#include <filesystem>
#include <iostream>
#include <unordered_set>
#include <vector>

#include <kiss_matcher/points/downsampling.hpp>
#include <kiss_matcher/points/point_cloud.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace kiss_matcher;

void savePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename) {
  if (cloud->empty()) {
    std::cerr << "Warning: Trying to save empty point cloud to " << filename << std::endl;
    return;
  }
  pcl::io::savePCDFileBinary(filename, *cloud);
  std::cout << "Saved " << cloud->size() << " points to " << filename << std::endl;
}

// Convert kiss_matcher::PointCloud to pcl::PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr convertCloudToPCL(const std::shared_ptr<PointCloud>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->resize(points->size());

  for (size_t i = 0; i < points->size(); ++i) {
    pcl_cloud->points[i].x = points->points[i].x();
    pcl_cloud->points[i].y = points->points[i].y();
    pcl_cloud->points[i].z = points->points[i].z();
  }
  return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertVectorToPCL(const std::vector<Eigen::Vector3f>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->resize(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    pcl_cloud->points[i].x = points[i](0);
    pcl_cloud->points[i].y = points[i](1);
    pcl_cloud->points[i].z = points[i](2);
  }
  return pcl_cloud;
}

template <typename Func>
double measure_execution_time(const std::string& label, Func func, int iterations = 10) {
  double total_time = 0.0;

  for (int i = 0; i < iterations; ++i) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();

    total_time += std::chrono::duration<double, std::milli>(end - start).count();
  }

  double average_time = total_time / iterations;
  std::cout << label << " average execution time: " << average_time << " ms" << std::endl;
  return average_time;
}

std::string get_output_filename(const std::string& input_pcd_file,
                                double resolution,
                                const std::string prefix = "") {
  std::filesystem::path file_path(input_pcd_file);
  std::string filename = file_path.stem().string();  // only get filename withoud ".pcd"

  std::ostringstream res_stream;
  res_stream << std::fixed << std::setprecision(2) << resolution;
  std::string res_string = res_stream.str();
  std::replace(res_string.begin(), res_string.end(), '.', '_');

  return prefix + filename + "_res_" + res_string + ".pcd";
}

int main(int argc, char** argv) {
  // E.g.,
  // ./downsampling_speed_comparison
  // /media/shapelim/UX960NVMe1/kiss-matcher-viz/vbr-colosseo/target.pcd 1.0 20
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <input_pcd_file> <resolution> <iterations>" << std::endl;
    return -1;
  }
  const std::string input_pcd_file = argv[1];
  const double resolution          = std::stod(argv[2]);
  const int iterations             = std::stoi(argv[3]);

  // Load points
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  int src_load_result = pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *points_pcl);

  // --------------------------------------------------------------------------------
  // convert PCL points to kiss_matcher PointCloud
  auto start_conversion = std::chrono::high_resolution_clock::now();
  auto points           = std::make_shared<PointCloud>();
  points->resize(points_pcl->size());

  for (size_t i = 0; i < points_pcl->size(); ++i) {
    points->points[i] = Eigen::Vector4d(
        points_pcl->points[i].x, points_pcl->points[i].y, points_pcl->points[i].z, 1.0);
  }
  auto end_conversion = std::chrono::high_resolution_clock::now();
  auto pointcloud_time =
      std::chrono::duration<double, std::milli>(end_conversion - start_conversion).count();
  std::cout << "Conversion time (My PointCloud): " << pointcloud_time << " ms" << std::endl;

  std::vector<Eigen::Vector3f> points_eigen;
  points_eigen.resize(points_pcl->size());
  for (size_t i = 0; i < points_pcl->size(); ++i) {
    points_eigen[i] = points_pcl->points[i].getVector3fMap();
  }
  auto end_eigen_conversion = std::chrono::high_resolution_clock::now();
  auto vector_time =
      std::chrono::duration<double, std::milli>(end_eigen_conversion - end_conversion).count();
  std::cout << "Conversion time (Eigen): " << vector_time << " ms" << std::endl;

  // --------------------------------------------------------------------------------
  // Voxelization Speed Benchmark
  // To ensure a fair comparison, we also include conversion times
  // (`pointcloud_time` and `vector_time`) in the measurements.
  // --------------------------------------------------------------------------------
  // Measure PCL VoxelGrid performance
  auto t_pcl = measure_execution_time(
      "PCL VoxelGrid",
      [&]() {
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(resolution, resolution, resolution);
        voxelgrid.setInputCloud(points_pcl);

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled =
            pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxelgrid.filter(*downsampled);
      },
      iterations);

  // Measure small_gicp voxelization performance
  auto t_tbb_with_cloud = measure_execution_time(
      "Small-GICP Voxelization (TBB)",
      [&]() { auto tbb_downsampled = VoxelgridSampling(*points, resolution); },
      iterations);
  std::cout << "\033[1;34m=> Total time taken with kiss_matcher::PointCloud: "
            << t_tbb_with_cloud + pointcloud_time << " ms\033[0m\n";

  auto t_tbb_with_vector = measure_execution_time(
      "std::vector<Eigen::Vector3f> Voxelization (TBB)",
      [&]() { VoxelgridSampling(points_eigen, resolution); },
      iterations);
  std::cout << "\033[1;32m=> Total time taken with vector: " << t_tbb_with_vector + vector_time
            << " ms\033[0m\n";
  // --------------------------------------------------------------------------------
  // Save cloud to pcd
  // --------------------------------------------------------------------------------
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(points_pcl);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_downsampled =
      pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxelgrid.filter(*pcl_downsampled);
  std::string pcl_output_filename = get_output_filename(input_pcd_file, resolution, "pcl_");
  savePointCloud(pcl_downsampled, pcl_output_filename);

  // Convert and save small_gicp downsampled cloud
  auto tbb_downsampled            = VoxelgridSampling(*points, resolution);
  auto pcl_tbb_downsampled        = convertCloudToPCL(tbb_downsampled);
  std::string tbb_output_filename = get_output_filename(input_pcd_file, resolution, "tbb_cloud_");
  savePointCloud(pcl_tbb_downsampled, tbb_output_filename);

  auto tbb_downsampled2            = VoxelgridSampling(points_eigen, resolution);
  auto pcl_tbb_downsampled2        = convertVectorToPCL(tbb_downsampled2);
  std::string tbb_output_filename2 = get_output_filename(input_pcd_file, resolution, "tbb_vec_");
  savePointCloud(pcl_tbb_downsampled2, tbb_output_filename2);

  return 0;
}
