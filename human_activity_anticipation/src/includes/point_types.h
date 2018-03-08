/* 
 * File:   point_types.h
 *
 * Created on June 21, 2011, 12:22 AM
 */

#ifndef POINT_TYPES_H
#define	POINT_TYPES_H
 #include <pcl/point_types.h>


namespace pcl

{
	struct PointXYZRGBS
	{
		PCL_ADD_POINT4D;
		float rgb;
		uint32_t segment;
	};

	struct PointXYZRGBDS
	{
		PCL_ADD_POINT4D;
		float rgb;
		float distance;
		uint32_t segment;
	};
    struct PointXYGRGBCam
    {
        PCL_ADD_POINT4D;
       float rgb;
        PCL_ADD_NORMAL4D;
       uint32_t cameraIndex;
       float distance;
             float curvature;

    };
    struct PointXYZRGBScore
    {
    		PCL_ADD_POINT4D;
    		float rgb;
    		double score;
    };

    struct PointXYZRGBCamSL
    {
        PCL_ADD_POINT4D;
 		union
  		{
    		struct
    		{
      			float rgb;
    		};
    		float data_c[4];
  		};
        uint32_t cameraIndex;
        float distance;
        uint32_t segment;
        uint32_t label;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYGRGBCam,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        )

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYZRGBCamSL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        (uint32_t, segment, segment)
        (uint32_t, label, label)
        )

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYZRGBS,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, segment, segment)
        )

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYZRGBDS,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (float, distance, distance)
        (uint32_t, segment, segment)
        )

        POINT_CLOUD_REGISTER_POINT_STRUCT(
                pcl::PointXYZRGBScore,
                (float, x, x)
                (float, y, y)
                (float, z, z)
                (float, rgb, rgb)
                (double, score, score)
                )

//PCL_INSTANTIATE_initTree(pcl::PointXYZRGBCamSL)

#endif	/* POINT_TYPES_H */

