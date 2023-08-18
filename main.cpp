#include<iostream>
#include<string>

#include "open3d/Open3D.h"

using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<PointCloud>;


namespace DataLoader{
    PointCloudPtr load_pointcloud_ply(const std::string &filename){
        PointCloudPtr pointcloud_ptr = PointCloudPtr(new PointCloud());
        open3d::io::ReadPointCloudOption option;

        if(open3d::io::ReadPointCloudFromPLY(filename, *pointcloud_ptr, option)){
            std::cout << "load point cloud done" << std::endl;

            if(false){
                std::vector< std::shared_ptr< const open3d::geometry::Geometry >> vis;
                vis.push_back(pointcloud_ptr);
                open3d::visualization::DrawGeometries(vis);
            }

        }else{
            std::cout << "load point cloud fail" << std::endl;
            exit(-1);
        }

        return pointcloud_ptr;
    }
};



class BallPivotAlgorithmImpl{
public:



};



class BallPivotAlgorithm{

public:

    BallPivotAlgorithm(){}
    ~BallPivotAlgorithm(){}

    void createMesh(){
    }

private:



};



int main(){
// load point cloud
    std::string pointcloud_ply_file = "./data/BunnyMesh.ply";
    PointCloudPtr pointcloud_ptr = DataLoader::load_pointcloud_ply(pointcloud_ply_file);

// define front


// find seed triangle


// ball pivot

    return 0;
}
