#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <optional>

#include <nanoflann.hpp>

#include "open3d/Open3D.h"

// define data type
using num_t = float;
using Point = Eigen::Vector3f;
using Normal = Eigen::Vector3d;
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

class Triangle{
public:
    Triangle(Edge e1, Edge e2, Edge e3)
    {
        edges[0] = e1;
        edges[1] = e2;
        edges[2] = e3;
    }

    IndexT getVertexIndex(const int &index){
        return std::get<0>(edges[index]);
    }

    Edge edges[3];
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
using kd_tree_t_ptr = std::shared_ptr<kd_tree_t>;


bool checkNormalConsistency(Triangle tri, PointCloudPtr pointcloud_ptr){

    // check inner product value between edge vertices.
    // we only check first 2 edges, because all vertices contained.
    for(uint ind = 0; ind < 2; ++ind){
        IndexT first = std::get<0>(tri.edges[ind]);
        IndexT second = std::get<1>(tri.edges[ind]);

        Normal norm1 = pointcloud_ptr->normals_[first];
        Normal norm2 = pointcloud_ptr->normals_[second];
        num_t dot_prod = norm1.dot(norm2);

        if(dot_prod < 0)
            return false;
    }

    return true;
}


class BallPivotAlgorithmImpl{
public:
    BallPivotAlgorithmImpl(const double &ball_radius) :
        used_count_(0),
        ball_radius_(ball_radius)
    {}

    std::optional<Triangle> find_seed_triangle(PointCloudPtr pointcloud_ptr, const IndexT &first_point_index=0){

        // This process lead inf loop bug... should be improved.
        while(1){
            // Pick any point that not yet used by the reconstructed triangulation.
            IndexT first_point = first_point_index;
            while(!used_[first_point]){
                first_point = rand() % point_count_;
            }

            // TODO: create set_used function.
            used_[first_point] = true;
            used_count_++;

            // all point is checked, reconstruction complete!
            if(used_count_ >= point_count_){
                return std::nullopt;
            }

            // Consider all pairs of points b, c in its neighborhood in order of distance from.
            int ret_count = 3; // of course contain itsself.
            size_t ret_index[ret_count];
            num_t out_dist_sqr[ret_count];
            nanoflann::KNNResultSet<num_t> resultSet(ret_count);
            resultSet.init((size_t *)&ret_index, (num_t *)&out_dist_sqr);

            kdtree_ptr_->findNeighbors(
                resultSet,
                (float*)pointcloud_ptr->points_[first_point].data(),
                nanoflann::SearchParams());

            // Build potential seed triangles a, b, c
            Edge e1(first_point, ret_index[1]);
            Edge e2(ret_index[1], ret_index[2]);
            Edge e3(ret_index[2], first_point);

            Triangle tri(e1, e2, e3);

            // Check that the triangle normal is consistent with the vertex normals, i.e., pointing outward.
            if(!checkNormalConsistency(tri, pointcloud_ptr)){
                continue;
            }

            // Test that ball with center in the outward halfspace touches all three vertices and contains no other data point.
            // calc ball center.
            // radius search.
            // if other point contains, reject
            if(false){
                continue;
            }

            return tri;

        }
        return std::nullopt;
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
        PC2KD pointcloud_adapter(pointcloud_ptr);
        kdtree_ptr_ = kd_tree_t_ptr(new kd_tree_t(3, pointcloud_adapter));

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

            // find_seed_triangle();
        }
    }

    int point_count_;
    int used_count_;

    double ball_radius_;

    std::vector<bool> used_;
    std::vector<bool> in_front_;
    std::vector<EdgeEmbd> front_;

    kd_tree_t_ptr kdtree_ptr_;
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
