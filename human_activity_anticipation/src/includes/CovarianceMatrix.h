

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

    inline void
    computeCentroid (const pcl::PointCloud<pcl::PointXYZRGB> &points, pcl::PointXYZ &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud                                                                                                                                                                            
      for (unsigned int i = 0; i < points.points.size (); i++)
	{
	  centroid.x += points.points.at (i).x;
	  centroid.y += points.points.at (i).y;
	  centroid.z += points.points.at (i).z;
	}

      centroid.x /= points.points.size ();
      centroid.y /= points.points.size ();
      centroid.z /= points.points.size ();
    }


    inline void
    computeCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &points, Eigen::Matrix3d &covariance_matrix, pcl::PointXYZ &centroid)
    {
      computeCentroid (points, centroid);

      // Initialize to 0                                                                                                                                                                                       
      covariance_matrix = Eigen::Matrix3d::Zero ();

      // Sum of outer products                                                                                                                                                                                 
      // covariance_matrix (k, i)  += points_c (j, k) * points_c (j, i);                                                                                                                                       
      for (unsigned int j = 0; j < points.points.size (); j++)
	{
	  covariance_matrix (0, 0) += (points.points[j].x - centroid.x) * (points.points[j].x - centroid.x);
	  covariance_matrix (0, 1) += (points.points[j].x - centroid.x) * (points.points[j].y - centroid.y);
	  covariance_matrix (0, 2) += (points.points[j].x - centroid.x) * (points.points[j].z - centroid.z);

	  covariance_matrix (1, 0) += (points.points[j].y - centroid.y) * (points.points[j].x - centroid.x);
	  covariance_matrix (1, 1) += (points.points[j].y - centroid.y) * (points.points[j].y - centroid.y);
	  covariance_matrix (1, 2) += (points.points[j].y - centroid.y) * (points.points[j].z - centroid.z);

	  covariance_matrix (2, 0) += (points.points[j].z - centroid.z) * (points.points[j].x - centroid.x);
	  covariance_matrix (2, 1) += (points.points[j].z - centroid.z) * (points.points[j].y - centroid.y);
	  covariance_matrix (2, 2) += (points.points[j].z - centroid.z) * (points.points[j].z - centroid.z);
	}
    }
