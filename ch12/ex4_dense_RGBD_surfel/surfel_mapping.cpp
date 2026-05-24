//
// Created by gaoxiang on 19-4-25.
// Modified for PCL 1.15 with correct API
//

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/search/kdtree.h>

using namespace std;

// typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloud;
typedef pcl::PointCloud<SurfelT>::Ptr SurfelCloudPtr;

SurfelCloudPtr reconstructSurface(
        const PointCloudPtr &input, float radius, int polynomial_order) {
    
    pcl::MovingLeastSquares<PointT, SurfelT> mls;
    
    // 使用 pcl::search::KdTree (PCL 1.15 正确用法)
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(radius * radius);
    
    // PCL 1.15 中使用 setPolynomialOrder，而不是 setPolynomialFit
    mls.setPolynomialOrder(polynomial_order);
    mls.setInputCloud(input);
    
    SurfelCloudPtr output(new SurfelCloud);
    mls.process(*output);
    
    return output;
}

pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr &surfels) {
    // Create search tree
    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);
    
    // Set parameters
    gp3.setSearchRadius(0.05);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    gp3.setNormalConsistency(true);
    
    // Get result
    gp3.setInputCloud(surfels);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);
    
    return triangles;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <pointcloud.pcd> [output_mesh.ply]" << endl;
        return 1;
    }
    
    // Load the points
    PointCloudPtr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
        cout << "Failed to load point cloud: " << argv[1] << endl;
        return 1;
    }
    cout << "Point cloud loaded, points: " << cloud->points.size() << endl;
    
    // Compute surface elements using MLS
    cout << "Computing MLS surface and normals ... " << endl;
    double mls_radius = 0.05;
    int polynomial_order = 2;
    auto surfels = reconstructSurface(cloud, mls_radius, polynomial_order);
    
    if (surfels->points.empty()) {
        cout << "MLS reconstruction failed!" << endl;
        return 1;
    }
    cout << "MLS completed, surfels: " << surfels->points.size() << endl;
    
    // Compute greedy surface triangulation
    cout << "Computing mesh via greedy triangulation ... " << endl;
    pcl::PolygonMeshPtr mesh = triangulateMesh(surfels);
    
    if (mesh->polygons.empty()) {
        cout << "Triangulation failed!" << endl;
        return 1;
    }
    cout << "Mesh generated with " << mesh->polygons.size() << " polygons" << endl;
    
    // Save mesh
    string output_file = (argc >= 3) ? argv[2] : "output_mesh.ply";
    pcl::io::savePLYFileBinary(output_file, *mesh);
    cout << "Mesh saved to: " << output_file << endl;
    
    return 0;
}