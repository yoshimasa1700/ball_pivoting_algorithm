#include<iostream>
#include<string>
#include<optional>

#include "open3d/Open3D.h"

using Point = Eigen::Vector3f;
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


using Edge = std::tuple<int, int>; // point index

using IndexT = unsigned int;


enum FLAG{
    ACTIVE,
    BOUNDARY,
    FROZEN
};


class EdgeEmbd{
public:
    Edge e;
    FLAG f;
};


class BallPivotAlgorithmImpl{
public:
    BallPivotAlgorithmImpl(const double &ball_radius) : ball_radius_(ball_radius){}

    void find_seed_triangle(){
        // decide point index.

        // get 2 neighbor

    }

    std::optional<IndexT> ball_pivot(){
        return 0;
    }

    void process(PointCloudPtr pointcloud_ptr){
        int point_count = pointcloud_ptr->points_.size();

        // define flag correspond to point cloud.
        used_ = std::vector<bool>(point_count, false);
        in_front_ = std::vector<bool>(point_count, false);

        // define front.
        front_ = std::vector<EdgeEmbd>();

        // build KD-Tree

        while(1){

            std::optional<IndexT> pivot_result = ball_pivot();

            if(pivot_result &&
               (used_[pivot_result.value()] || in_front_[pivot_result.value()])
                ){
                // output triangle

                // join

                // glue(remove coincident edges)

            }else{
                // mark as boundary
            }

            find_seed_triangle();
        }
    }

    double ball_radius_;

    std::vector<bool> used_;
    std::vector<bool> in_front_;

    std::vector<EdgeEmbd> front_;
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
