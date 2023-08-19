#include <cstdlib>
#include <iostream>
#include <string>
#include <optional>

#include <nanoflann.hpp>

#include "open3d/Open3D.h"

// define data type
using num_t = float;
using Point = Eigen::Vector3f;
using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<PointCloud>;
using IndexT = unsigned int;
using Edge = std::tuple<IndexT, IndexT>; // point index

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


template <typename Derived>
struct PointCloudAdaptor
{
    using coord_t = num_t;

    const Derived& obj;  //!< A const ref to the data set origin

    /// The constructor that sets the data set source
    PointCloudAdaptor(const Derived& obj_) : obj(obj_) {}

    /// CRTP helper method
    inline const Derived& derived() const { return obj; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const
    {
        return derived()->points_.size();
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline coord_t kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return derived()->points_[idx][0];
        else if (dim == 1)
            return derived()->points_[idx][1];
        else
            return derived()->points_[idx][2];
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }

};  // end of PointCloudAdaptor


using PC2KD = PointCloudAdaptor<PointCloudPtr>;
using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<num_t, PC2KD>, PC2KD, 3 /* dim */
    >;


auto do_knn_search = [](const kd_tree_t& index) {
    // do a knn search
    const size_t                   num_results = 1;
    size_t                         ret_index;
    num_t                          out_dist_sqr;
    nanoflann::KNNResultSet<num_t> resultSet(num_results);
    num_t                          query_pt[3] = {0.5, 0.5, 0.5};

    resultSet.init(&ret_index, &out_dist_sqr);
    index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams());

    std::cout << "knnSearch(nn=" << num_results << "): \n";
    std::cout << "ret_index=" << ret_index
              << " out_dist_sqr=" << out_dist_sqr << std::endl;
};

class BallPivotAlgorithmImpl{
public:
    BallPivotAlgorithmImpl(const double &ball_radius) : ball_radius_(ball_radius){}

    void find_seed_triangle(const IndexT &first_point_index=0){
        // Pick any point a  not yet used by the reconstructed triangulation.
        IndexT first_point = first_point_index;
        while(!used_[first_point]){
            first_point = rand() % point_count_;
        }

        // Consider all pairs of points b, c in its neighborhood in order of distance from.

        // Build potential seed triangles a, b, c

        // Check that the triangle normal is consistent with the vertex normals, i.e., pointing outward.

        // Test that ball with center in the outward halfspace touches all three vertices and contains no other data point.
    }

    std::optional<IndexT> ball_pivot(){
        return 0;
    }

    void process(PointCloudPtr pointcloud_ptr){
        point_count_ = pointcloud_ptr->points_.size();

        // define flag correspond to point cloud.
        used_ = std::vector<bool>(point_count_, false);
        in_front_ = std::vector<bool>(point_count_, false);

        // define front.
        front_ = std::vector<EdgeEmbd>();

        // build KD-Tree
        PointCloudAdaptor<PointCloudPtr> pointcloud_adapter(pointcloud_ptr);



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

    int point_count_;

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
