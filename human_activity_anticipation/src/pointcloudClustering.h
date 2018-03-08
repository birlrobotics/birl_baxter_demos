

#include <stdint.h>

#include "includes/color.cpp"
#include "pcl/ModelCoefficients.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/search/impl/organized.hpp"
//#include "pcl/search/kdtree/tree_types.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
//#include "pcl/search/kdtree/impl/tree_types.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>

#include "pcl/io/pcd_io.h"
#include "includes/point_types.h"
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "includes/CombineUtils.h"
#include "includes/CovarianceMatrix.h"

//#include <sensor_msgs/point_cloud_conversion.h>
//#include <point_cloud_mapping/geometry/nearest.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBCamSL PointOutT;

typedef pcl::search::KdTree<PointOutT> KdTree;
typedef pcl::search::KdTree<PointOutT>::Ptr KdTreePtr;

/* 
Extract the clusters based on location and normals
 */
void extractEuclideanClusters(
        const pcl::PointCloud<PointOutT> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
        const boost::shared_ptr<KdTree > &tree,
        float tolerance, std::vector<pcl::PointIndices> &clusters, double eps_angle,
        unsigned int min_pts_per_cluster = 1,
        unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ()) {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    float adjTolerance = 0;
    if (tree->getInputCloud()->points.size() != cloud.points.size()) {
        //ROS_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud()->points.size(), cloud.points.size());
        return;
    }
    if (cloud.points.size() != normals.points.size()) {
        //ROS_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!", cloud.points.size(), normals.points.size());
        return;
    }
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed(cloud.points.size(), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (processed[i])
            continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back(i);
        processed[i] = true;

        int cnt = 0;

        while (sq_idx < (int) seed_queue.size()) {
            cnt++;
            //ROS_INFO ("i = %d, cnt = %d", i , cnt);

            // Search for sq_idx
            //float distance = sqrt(cloud.points[seed_queue[sq_idx]].x*cloud.points[seed_queue[sq_idx]].x + cloud.points[seed_queue[sq_idx]].y*cloud.points[seed_queue[sq_idx]].y + cloud.points[seed_queue[sq_idx]].z + cloud.points[seed_queue[sq_idx]].z);
            adjTolerance = cloud.points[seed_queue[sq_idx]].distance * tolerance;
            //cout << "adj Tolerance : "<< adjTolerance << endl;
            //adjTolerance = tolerance;
            if (!tree->radiusSearch(seed_queue[sq_idx], adjTolerance, nn_indices, nn_distances)) {
                sq_idx++;
                continue;
            }

            for (size_t j = 1; j < nn_indices.size(); j++) // nn_indices[0] should be sq_idx
            {
                if (processed[nn_indices[j]]) // Has this point been processed before ?
                    continue;

                //processed[nn_indices[j]] = true;
                // [-1;1]
                double dot_p =
                        normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                        normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                        normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
                //cout << "angle :" << fabs (acos (dot_p)) << endl;
                if (fabs(acos(dot_p)) < eps_angle) {
                    processed[nn_indices[j]] = true;
                    seed_queue.push_back(nn_indices[j]);
                }
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster) {
            pcl::PointIndices r;
            r.indices.resize(seed_queue.size());
            for (size_t j = 0; j < seed_queue.size(); j++)
                r.indices[j] = seed_queue[j];

            sort(r.indices.begin(), r.indices.end());
            //r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = cloud.header;
            //ROS_INFO ("cluster of size %d data point\n ",r.indices.size());
            clusters.push_back(r);
        }
    }
}

/* 
Extract the clusters based on location 
 */
void extractEuclideanClusters(
        const pcl::PointCloud<PointOutT> &cloud,
        const boost::shared_ptr<KdTree > &tree,
        float tolerance, std::vector<pcl::PointIndices> &clusters,
        unsigned int min_pts_per_cluster = 1,
        unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ()) {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    float adjTolerance = 0;
    if (tree->getInputCloud()->points.size() != cloud.points.size()) {
        //ROS_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!", tree->getInputCloud()->points.size(), cloud.points.size());
        return;
    }

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed(cloud.points.size(), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        if (processed[i])
            continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back(i);
        processed[i] = true;

        int cnt = 0;

        while (sq_idx < (int) seed_queue.size()) {
            cnt++;
            //ROS_INFO ("i = %d, cnt = %d", i , cnt);

            // Search for sq_idx
            //float distance = sqrt(cloud.points[seed_queue[sq_idx]].x*cloud.points[seed_queue[sq_idx]].x + cloud.points[seed_queue[sq_idx]].y*cloud.points[seed_queue[sq_idx]].y + cloud.points[seed_queue[sq_idx]].z + cloud.points[seed_queue[sq_idx]].z);
            adjTolerance = cloud.points[seed_queue[sq_idx]].distance * tolerance;
            //adjTolerance = tolerance;
            if (!tree->radiusSearch(seed_queue[sq_idx], adjTolerance, nn_indices, nn_distances)) {
                sq_idx++;
                continue;
            }

            for (size_t j = 0; j < nn_indices.size(); j++) // nn_indices[0] should be sq_idx
            {
                if (processed[nn_indices[j]]) // Has this point been processed before ?
                    continue;



                processed[nn_indices[j]] = true;
                seed_queue.push_back(nn_indices[j]);

            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster) {
            pcl::PointIndices r;
            r.indices.resize(seed_queue.size());
            for (size_t j = 0; j < seed_queue.size(); j++)
                r.indices[j] = seed_queue[j];

            sort(r.indices.begin(), r.indices.end());
            //r.indices.erase (unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

            r.header = cloud.header;
            //ROS_INFO ("cluster of size %d data point\n ",r.indices.size());
            clusters.push_back(r);
        }
    }
}

void convert(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointOutT> &cloud_out) {
    // cloud_out.header = cloud_in.header;
    //  clusters[i].header.stamp = ros::Time(0);
    cloud_out.points.resize(cloud_in.points.size());
    for (size_t j = 0; j < cloud_in.points.size(); j++) {
        
        cloud_out.points[j].x = cloud_in.points[j].x;
        cloud_out.points[j].y = cloud_in.points[j].y;
        cloud_out.points[j].z = cloud_in.points[j].z;
        cloud_out.points[j].rgb = cloud_in.points[j].rgb;
        cloud_out.points[j].cameraIndex = 1;
        cloud_out.points[j].distance = sqrt(cloud_in.points[j].x * cloud_in.points[j].x + cloud_in.points[j].y * cloud_in.points[j].y + cloud_in.points[j].z + cloud_in.points[j].z);
        ;
        cloud_out.points[j].segment = 0;
        cloud_out.points[j].label = 0;
    }

}

int getClustersFromPointCloud2(const pcl::PointCloud<PointOutT> &cloud_objects,
        const std::vector<pcl::PointIndices> &clusters2,
        std::vector<pcl::PointCloud<PointT> > &clusters) {
    int max_cluster_size = 0;
    int max_cluster_index = 0;
    clusters.resize(clusters2.size());
    for (size_t i = 0; i < clusters2.size(); i++) {
        clusters[i].header.frame_id = cloud_objects.header.frame_id;
        //  clusters[i].header.stamp = ros::Time(0);
        clusters[i].points.resize(clusters2[i].indices.size());
        if (clusters2[i].indices.size() > max_cluster_size) {
            max_cluster_index = i;
            max_cluster_size = clusters2[i].indices.size();
        }
        for (size_t j = 0; j < clusters[i].points.size(); j++) {
            clusters[i].points[j].x = cloud_objects.points[clusters2[i].indices[j]].x;
            clusters[i].points[j].y = cloud_objects.points[clusters2[i].indices[j]].y;
            clusters[i].points[j].z = cloud_objects.points[clusters2[i].indices[j]].z;
            clusters[i].points[j].rgb = cloud_objects.points[clusters2[i].indices[j]].rgb;
        }


    }
    return max_cluster_index;
}

int getClusters(pcl::PointCloud<PointOutT> &cloud, std::vector<pcl::PointCloud<PointT> > &clustersOut, std::vector<pcl::PointIndices> &clusterInds, bool useNormals) {

    int min_cluster_size_ = 100;
    int min_pts_per_cluster = 0;
    int max_pts_per_cluster = 3000000;
    int number_neighbours = 50;
    float radius = 0.05;//0.025; // 0.01*10 ;//0.025*1000;//0.01*1000;// 0.025 
    float angle = 0.52; //0.52; 




    KdTreePtr clusters_tree_  (new pcl::search::KdTree<PointOutT> ());




    pcl::PointCloud<PointOutT>::Ptr cloud_ptr(new pcl::PointCloud<PointOutT > (cloud));
    // Cluster the points



    //clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > ();
    //initTree(0, clusters_tree_);
    clusters_tree_->setInputCloud(cloud_ptr);
    //clusters_tree_->setInputCloud (cloud_filtered );// ,indicesp);
    // extractEuclideanClusters ( *cloud_filtered, clusters_tree_, tolerance, clusters,  min_pts_per_cluster, max_pts_per_cluster);

    if (useNormals) {
        pcl::search::KdTree<PointOutT>::Ptr normals_tree_ (new pcl::search::KdTree<PointOutT> ());
        pcl::NormalEstimation<PointOutT, pcl::Normal> n3d_;
        //normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > ();
        n3d_.setKSearch(number_neighbours);
        n3d_.setSearchMethod(normals_tree_);
        pcl::PointCloud<pcl::Normal> cloud_normals;
        n3d_.setInputCloud(cloud_ptr);
        n3d_.compute(cloud_normals);
        //pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr =  boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);
        extractEuclideanClusters(*cloud_ptr, cloud_normals, clusters_tree_, radius, clusterInds, angle, min_pts_per_cluster, max_pts_per_cluster);
    } else {
        extractEuclideanClusters(*cloud_ptr, clusters_tree_, radius, clusterInds, min_pts_per_cluster, max_pts_per_cluster);
    }
    //extractEuclideanClusters ( *cloud_filtered, *cloud_normals_ptr, clusters_tree_, radius, clusters, angle, min_pts_per_cluster, max_pts_per_cluster);
    //ROS_INFO("Number of clusters found matching the given constraints: %d.", (int) clusterInds.size());


    int max_cluster_index = getClustersFromPointCloud2(*cloud_ptr, clusterInds, clustersOut);
    //int max_cluster_index = getClustersFromPointCloud2(*cloud_filtered, clusters, clustersOut);

    return max_cluster_index;

}

void getMaxCluster(pcl::PointCloud<PointT> &cloud_in) {


    // convert to PointXYZRGBCamSL format
    pcl::PointCloud<PointOutT> cloud;
    convert(cloud_in, cloud);

    std::vector<pcl::PointCloud<PointT> > clustersOut;
    std::vector<pcl::PointIndices> clusterInds;
    int max_cluster_index = getClusters(cloud, clustersOut, clusterInds, false);
    //int max_cluster_index = getClustersFromPointCloud2(*cloud_filtered, clusters, clustersOut);
    cloud_in = clustersOut.at(max_cluster_index);


}

void getMaxCluster(pcl::PointCloud<PointT> &cloud_in, pcl::PointIndices & indices) {


    // convert to PointXYZRGBCamSL format
    pcl::PointCloud<PointOutT> cloud;
    convert(cloud_in, cloud);

    std::vector<pcl::PointCloud<PointT> > clustersOut;
    std::vector<pcl::PointIndices> clusterInds;
    int max_cluster_index = getClusters(cloud, clustersOut, clusterInds, false);
    //int max_cluster_index = getClustersFromPointCloud2(*cloud_filtered, clusters, clustersOut);
    cloud_in = clustersOut.at(max_cluster_index);
    indices = clusterInds.at(max_cluster_index);

}

void computeCentroid(pcl::PointXYZ &centroid, pcl::PointCloud<PointT> &cloud) {

    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;
    for (size_t i = 0; i < cloud.points.size(); i++) {
        centroid.x += cloud.points.at(i).x;
        centroid.y += cloud.points.at(i).y;
        centroid.z += cloud.points.at(i).z;
    }
    centroid.x = centroid.x / cloud.points.size();
    centroid.y = centroid.y / cloud.points.size();
    centroid.z = centroid.z / cloud.points.size();

}

void getNormal(pcl::PointCloud<PointT> &cloud, Eigen::Vector3d &normal) {
    Eigen::Matrix3d eigen_vectors;
    Eigen::Vector3d eigen_values;
    //sensor_msgs::PointCloud2 cloudMsg2;
   // pcl::toROSMsg(cloud, cloudMsg2);
   // sensor_msgs::PointCloud cloudMsg;
  //  sensor_msgs::convertPointCloud2ToPointCloud(cloudMsg2, cloudMsg);
    pcl::PointXYZ centroid;
    computeCentroid(centroid, cloud);
    //cloud_geometry::nearest::computePatchEigenNormalized(cloudMsg, eigen_vectors, eigen_values, centroid);
    Eigen::Matrix3d covariance_matrix;
    computeCovarianceMatrix(cloud, covariance_matrix,centroid);
    for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int j = 0; j < 3; j++) {
            covariance_matrix(i, j) /= static_cast<double> (cloud.points.size());
        }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm(covariance_matrix);
    eigen_values = ei_symm.eigenvalues();
    eigen_vectors = ei_symm.eigenvectors();


    float minEigV = FLT_MAX;

    for (int i = 0; i < 3; i++) {

        //      cout<<"eig value:"<<eigen_values(i)<<endl;
        if (minEigV > eigen_values(i)) {
            minEigV = eigen_values(i);
            //    cout<<"min eig value:"<<minEigV<<endl;
            normal = eigen_vectors.col(i);
            // check the angle with line joining the centroid to origin
            VectorG centroid_(centroid.x, centroid.y, centroid.z);
            VectorG camera(0, 0, 0);
            VectorG cent2cam = camera.subtract(centroid_);
            VectorG normal_(normal[0], normal[1], normal[2]);
            if (normal_.dotProduct(cent2cam) < 0) {
                // flip the sign of the normal
                normal[0] = -normal[0];
                normal[1] = -normal[1];
                normal[2] = -normal[2];
            }
        }
    }

}

void filterForTable(pcl::PointCloud<PointOutT> &cloud, pcl::PointIndices &tablePoints) {

    pcl::PointCloud<PointOutT> cloud_out;

    for (int i = 0; i < cloud.size(); i++) {
       // cout << i << ":" << cloud.points[i].distance << ", z:"<< cloud.points[i].z  << endl;
        if (cloud.points[i].distance < 2000 && cloud.points[i].distance > 550 && cloud.points[i].z < 100 && cloud.points[i].z > -100) {
            tablePoints.indices.push_back(i);
        }

    }
    // copy header
    cloud_out.header = cloud.header;
    cloud_out.sensor_origin_ = cloud.sensor_origin_;
    cloud_out.sensor_orientation_ = cloud.sensor_orientation_;
    cloud_out.height = 1;
    cloud_out.width = tablePoints.indices.size();
    cloud_out.points.resize(cloud_out.width * cloud_out.height);
    for (size_t i = 0; i < tablePoints.indices.size(); i++) {
        cloud_out.points.at(i) = cloud.points.at(tablePoints.indices.at(i));
    }
    cloud = cloud_out;
    cout << "num points in filtered cloud : " << cloud.points.size() << endl;
}

void getTableInds(pcl::PointCloud<PointT> &cloud_in, pcl::PointIndices &cloudInds) {


    // convert to PointXYZRGBCamSL format
    pcl::PointCloud<PointOutT> cloud;
    convert(cloud_in, cloud);
    // full cloud segmentation take a lot of time
    // filter out points too far and not near table 
    pcl::PointIndices tablePoints;
    filterForTable(cloud, tablePoints);
    std::vector<pcl::PointCloud<PointT> > clustersOut;
    //std::vector<pcl::PointCloud<PointT> > clustersOutSorted;
    std::vector<pcl::PointIndices> clusterInds;
    int maxSize = 0;
    int max_cluster_index = getClusters(cloud, clustersOut, clusterInds, true);
    pcl::PointIndices tmpInds;
    Eigen::Vector3d normal;
    for (size_t i = 0; i < clustersOut.size(); i++) {
        if (clustersOut.at(i).size() > 1000) {
            getNormal(clustersOut.at(i), normal);
            cout << "cluster: " << i << " size:" << clustersOut.at(i).points.size() << " normal: " << normal[0] << "," << normal[1] << "," << normal[2] << endl;
            if ((normal[2] > 0.9 || normal[2] < -0.9  )&& clustersOut.at(i).points.size() > maxSize) {
                maxSize = clustersOut.at(i).points.size();
                cout << "normal is " << normal[0] << "," << normal[1] << "," << normal[2] << endl;
                tmpInds = clusterInds.at(i);
            }
        }
    }
    //cloud_in  = clustersOut.at(max_cluster_index);
    // get the original indicies
    for (size_t i = 0; i < tmpInds.indices.size(); i++) {
        cloudInds.indices.push_back(tablePoints.indices.at(tmpInds.indices.at(i)));
    }

}
/* ]--- */
