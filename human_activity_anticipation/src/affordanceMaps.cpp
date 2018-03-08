/* 
 * File:   affordanceMap.cpp
 * Author: hema
 *
 * Created on August 29, 2011, 6:16 PM
 */

#include "affordanceMaps.h"

int Frame::FrameNum = 0;

pcl::PointXYZRGBScore affordanceMaps::getImagePoint(pcl::PointXYZRGBScore  p,TransformG globalTransform ){
	pcl::PointXYZ p1;
	p1.x = p.x ; p1.y = p.y; p1.z = p.z;
    globalTransform.transformPointInPlace(p1);
    pcl::PointXYZRGBScore rp;
    if(p1.y != 0.0){
        rp.x = int(((p1.x * (640.0/1.1147))/p1.y) + 320);
        rp.y = int(240 - ((p1.z * (480.0/0.8336))/p1.y));
    }
    rp.score = p.score;
    return rp;
}

void affordanceMaps::saveFloatImage ( const char* filename, const IplImage * image )
{
  IplImage * saveImage = cvCreateImage ( cvGetSize ( image ),
                                             IPL_DEPTH_32F, 3 );
  cvConvertScale ( image, saveImage, 255, 0 );
  cvSaveImage( filename, saveImage);
  cvReleaseImage ( &saveImage );
}


void affordanceMaps::saveImage(pcl::PointCloud<PointT> & cloud, string imagename) {
        CvSize size;
        int thickness = 8;
        int lineType = 8;
        size.height = 480;
        size.width = 640;
        IplImage * image = cvCreateImage(size, IPL_DEPTH_32F, 3);
        //assert(cloud.size() == size.width * size.height);
        PointT tmp;
        for (int x = 0; x < size.width; x++)
            for (int y = 0; y < size.height; y++) {
                int index = ( x) + ( y) * 640;
                tmp = cloud.points[index];
                ColorRGB tmpColor(tmp.rgb);
                CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b; //float(IMAGE[obj.minX + x][obj.minY + y][0]) /255 ;// tmpColor.b;
                CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g; //float(IMAGE[obj.minX + x][obj.minY + y][1])/255 ; //tmpColor.g;
                CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r; //float(IMAGE[obj.minX + x][obj.minY + y][2])/255 ;//tmpColor.r;
            }

        string filename = imagename;




        //cvLine( image, cvPoint(10, 40), cvPoint(100,200 ),cv::Scalar( 1,1,1 ),thickness,lineType );
        saveFloatImage(filename.c_str(), image);

        cvReleaseImage(&image);


    }

double affordanceMaps::getDistanceSqrBwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZ &p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}


double affordanceMaps::getDistanceSqrBwPoints(pcl::PointXYZ &p1, pcl::PointXYZ p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}

double affordanceMaps::getDistanceSqrBwPoints(pcl::PointXYZ &p1, pcl::PointXYZRGBScore &p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}
double affordanceMaps::getDistanceSqrBwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZRGBScore &p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}

double affordanceMaps::getPdf_VM(float alpha, float mean, float kappa){
	double normc[] = {1.257083e-001, 6.981750e-002, 3.260842e-002, 1.408211e-002, 5.842720e-003, 2.367165e-003, 9.440136e-004, 3.722364e-004, 1.455346e-004, 5.652378e-005, 2.183648e-005, 8.399154e-006, 3.218861e-006, 1.229769e-006, 4.685860e-007, 1.781360e-007, 6.758257e-008, 2.559414e-008, 9.677314e-009, 3.653839e-009, 1.377798e-009, 5.189390e-010, 1.952481e-010, 7.338999e-011, 2.756139e-011, 1.034214e-011, 3.877855e-012, 1.453011e-012, 5.440811e-013, 2.036083e-013, 7.615204e-014, 2.846674e-014, 1.063600e-014, 3.972064e-015, 1.482734e-015, 5.532615e-016, 2.063609e-016, 7.694203e-017, 2.867789e-017, 1.068529e-017, 3.980041e-018, 1.482034e-018, 5.517013e-019, 2.053199e-019, 7.639141e-020, 2.841512e-020, 1.056698e-020, 3.928733e-021, 1.460357e-021, 5.427179e-022, 2.016515e-022, 7.491080e-023, 2.782315e-023, 1.033214e-023, 3.836178e-024, 1.424082e-024, 5.285683e-025, 1.961552e-025, 7.278361e-026, 2.700252e-026, 1.001646e-026, 3.715055e-027, 1.377715e-027, 5.108557e-028, 1.894016e-028, 7.021297e-029, 2.602560e-029, 9.645738e-030, 3.574562e-030, 1.324537e-030, 4.907507e-031, 1.818086e-031, 6.734812e-032, 2.494570e-032, 9.239018e-033, 3.421505e-033, 1.266983e-033, 4.691236e-034, 1.736873e-034, 6.430039e-035, 2.380264e-035, 8.810557e-036, 3.260987e-036, 1.206877e-036, 4.466282e-037, 1.652719e-037, 6.115362e-038, 2.262646e-038, 8.371104e-039, 3.096859e-039, 1.145600e-039, 4.237584e-040, 1.567393e-040, 5.797117e-041, 2.143984e-041, 7.928791e-042, 2.932031e-042, 1.084194e-042, 4.008878e-043, 1.482232e-043, 5.480093e-044, 2.025994e-044, 7.489751e-045, 2.768701e-045, 1.023445e-045, 3.782974e-046, 1.398243e-046, 5.167886e-047, 1.909961e-047, 7.058584e-048, 2.608512e-048, 9.639405e-049, 3.561970e-049, 1.316174e-049, 4.863170e-050, 1.796839e-050, 6.638694e-051, 2.452676e-051, 9.061124e-052, 3.347407e-052, 1.236573e-052, 4.567898e-053, 1.687323e-053, 6.232547e-054, 2.302071e-054, 8.502723e-055, 3.140390e-055, 1.159833e-055, 4.283456e-056, 1.581903e-056, 5.841879e-057, 2.157310e-057, 7.966363e-058, 2.941679e-058, 1.086221e-058, 4.010785e-059, 1.480910e-059, 5.467846e-060, 2.018796e-060, 7.453452e-061, 2.751765e-061, 1.015907e-061, 3.750474e-062, 1.384547e-062, 5.111148e-063, 1.886770e-063, 6.964811e-064, 2.570926e-064, 9.489860e-065, 3.502840e-065, 1.292918e-065, 4.772130e-066, 1.761344e-066, 6.500796e-067, 2.399274e-067, 8.854908e-068, 3.267980e-068, 1.206051e-068, 4.450856e-069, 1.642527e-069, 6.061403e-070, 2.236791e-070, 8.254091e-071, 3.045825e-071, 1.123913e-071, 4.147172e-072, 1.530255e-072, 5.646348e-073, 2.083357e-073, 7.686919e-074, 2.836177e-074, 1.046422e-074, 3.860762e-075, 1.424400e-075, 5.255132e-076, 1.938779e-076, 7.152631e-077, 2.638739e-077, 9.734647e-078, 3.591180e-078, 1.324791e-078, 4.887097e-079, 1.802802e-079, 6.650258e-080, 2.453141e-080, 9.048993e-081, 3.337887e-081, 1.231223e-081, 4.541464e-082, 1.675131e-082, 6.178680e-083, 2.278960e-083, 8.405658e-084, 3.100280e-084, 1.143469e-084, 4.217373e-085, 1.555443e-085, 5.736677e-086, 2.115735e-086, 7.802909e-087, 2.877706e-087, 1.061282e-087, 3.913904e-088, 1.443391e-088, 5.322954e-089, 1.962982e-089, 7.238934e-090, 2.669487e-090, 9.844102e-091, 3.630107e-091, 1.338621e-091, 4.936182e-092, 1.820203e-092, 6.711870e-093, 2.474928e-093, 9.125926e-094, 3.365012e-094, 1.240771e-094, 4.575011e-095, 1.686895e-095, 6.219844e-096, 2.293330e-096, 8.455690e-097, 3.117649e-097, 1.149479e-097, 4.238094e-098, 1.562557e-098, 5.760988e-099, 2.123997e-099, 7.830808e-100, 2.887056e-100, 1.064387e-100, 3.924102e-101, 1.446694e-101, 5.333463e-102, 1.966246e-102, 7.248739e-103, 2.672288e-103, 9.851451e-104, 3.631728e-104, 1.338822e-104, 4.935468e-105, 1.819409e-105, 6.707002e-106, 2.472424e-106, 9.114103e-107, 3.359706e-107, 1.238469e-107, 4.565258e-108, 1.682837e-108, 6.203192e-109, 2.286573e-109, 8.428522e-110, 3.106808e-110, 1.145181e-110, 4.221144e-111, 1.555905e-111, 5.734991e-112, 2.113874e-112, 7.791521e-113, 2.871853e-113, 1.058519e-113, 3.901507e-114, 1.438013e-114, 5.300173e-115, 1.953504e-115, 7.200047e-116, 2.653709e-116, 9.780665e-117, 3.604794e-117, 1.328585e-117, 4.896610e-118, 1.804673e-118, 6.651182e-119, 2.451298e-119, 9.034218e-120, 3.329524e-120, 1.227074e-120, 4.522273e-121, 1.666633e-121, 6.142148e-122, 2.263591e-122, 8.342049e-123, 3.074290e-123, 1.132959e-123, 4.175237e-124, 1.538669e-124, 5.670309e-125, 2.089611e-125, 7.700551e-126, 2.837759e-126, 1.045747e-126, 3.853675e-127, 1.420107e-127, 5.233167e-128, 1.928437e-128, 7.106309e-129, 2.618666e-129, 9.649698e-130, 3.555862e-130, 1.310309e-130, 4.828363e-131, 1.779196e-131, 6.556099e-132, 2.415821e-132, 8.901878e-133, 3.280169e-133, 1.208672e-133, 4.453674e-134, 1.641066e-134, 6.046883e-135, 2.228100e-135, 8.209859e-136, 3.025063e-136, 1.114631e-136, 4.107005e-137, 1.513273e-137, 5.575803e-138, 2.054448e-138, 7.569739e-139, 2.789103e-139, 1.027652e-139, 3.786390e-140, 1.395091e-140, 5.140174e-141, 1.893874e-141, 6.977857e-142, 2.570935e-142, 9.472360e-143, 3.489983e-143, 1.285838e-143, 4.737480e-144, 1.745446e-144, 6.430779e-145, 2.369293e-145, 8.729150e-146, 3.216054e-146, 1.184875e-146, 4.365361e-147, 1.608295e-147, 5.925288e-148, 2.182988e-148, 8.042505e-149, 2.962985e-149, 1.091606e-149, 4.021613e-150, 1.481607e-150, 5.458380e-151, 2.010911e-151, 7.408326e-152, 2.729264e-152, 1.005470e-152, 3.704173e-153, 1.364619e-153, 5.027245e-154, 1.852025e-154, 6.822788e-155, 2.513479e-155, 9.259486e-156, 3.411119e-156, 1.256623e-156, 4.629262e-157, 1.705362e-157, 6.282319e-158, 2.314311e-158, 8.525536e-159, 3.140654e-159, 1.156956e-159, 4.261987e-160, 1.570022e-160, 5.783593e-161, 2.130532e-161, 7.848325e-162, 2.891108e-162, 1.065001e-162, 3.923144e-163, 1.445163e-163, 5.323509e-164, 1.961000e-164, 7.223633e-165, 2.660923e-165, 9.801834e-166, 3.610613e-166, 1.330004e-166, 4.899184e-167, 1.804650e-167, 6.647534e-168, 2.448651e-168, 9.019691e-169, 3.322424e-169, 1.223818e-169, 4.507934e-170, 1.660491e-170, 6.116378e-171, 2.252945e-171, 8.298614e-172, 3.056744e-172, 1.125930e-172, 4.147267e-173, 1.527606e-173, 5.626774e-174, 2.072555e-174, 7.633989e-175, 2.811872e-175, 1.035710e-175, 3.814868e-176, 1.405140e-176, 5.175571e-177, 1.906320e-177, 7.021531e-178, 2.586227e-178, 9.525773e-179, 3.508589e-179, 1.292300e-179, 4.759849e-180, 1.753160e-180, 6.457268e-181, 2.378344e-181, 8.759906e-182, 3.226435e-182, 1.188352e-182, 4.376895e-183, 1.612077e-183, 5.937512e-184, 2.186864e-184, 8.054489e-185, 2.966559e-185, 1.092614e-185, 4.024197e-186, 1.482145e-186, 5.458847e-187, 2.010527e-187, 7.404877e-188, 2.727248e-188, 1.004454e-188, 3.699428e-189, 1.362504e-189, 5.018108e-190, 1.848166e-190, 6.806766e-191, 2.506915e-191, 9.232881e-192, 3.400429e-192, 1.252360e-192, 4.612363e-193, 1.698700e-193, 6.256174e-194, 2.304092e-194, 8.485742e-195, 3.125206e-195, 1.150977e-195, 4.238901e-196, 1.561129e-196, 5.749413e-197, 2.117420e-197, 7.798114e-198, 2.871911e-198, 1.057673e-198, 3.895209e-199, 1.434528e-199, 5.283071e-200, 1.945641e-200, 7.165361e-201, 2.638836e-201, 9.718194e-202, 3.578968e-202, 1.318042e-202, 4.853995e-203, 1.787593e-203, 6.583197e-204, 2.424399e-204, 8.928335e-205, 3.288030e-205, 1.210877e-205, 4.459268e-206, 1.642200e-206, 6.047661e-207, 2.227142e-207, 8.201769e-208, 3.020412e-208, 1.112305e-208, 4.096194e-209, 1.508469e-209, 5.555092e-210, 2.045715e-210, 7.533524e-211, 2.774280e-211, 1.021648e-211, 3.762285e-212, 1.385483e-212, 5.102107e-213, 1.878871e-213, 6.919006e-214, 2.547942e-214, 9.382841e-215, 3.455241e-215, 1.272393e-215, 4.685583e-216, 1.725460e-216, 6.353974e-217, 2.339834e-217, 8.616358e-218, 3.172938e-218, 1.168419e-218, 4.302638e-219, 1.584419e-219, 5.834512e-220, 2.148514e-220, 7.911720e-221, 2.913418e-221, 1.072837e-221, 3.950610e-222};
    vector<double> normconstants (normc,normc+ sizeof(normc)/sizeof(double));
    //for (size_t i = 0 ; i < normconstants.size(); i ++){
    //	cout << i << ":" << normconstants.at(i) << endl;
    //}
    kappa = round(kappa);
    double normconstant = normconstants.at(kappa-1);

    double val = normconstant * exp(kappa * cos(alpha - mean));
    if(val < DBL_MIN){val = 0;}
    if(val != val){val = 0;}
    if(std::numeric_limits<double>::has_infinity &&
    		val == std::numeric_limits<double>::infinity()){val = 0;}
    return val;
}

void affordanceMaps::pourabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples ,int obj, int objRef ){
	//Parameters: N: 32, meanH : -190.618 , sigmaH : 58.1901, meanD : 91128.7 , sigmaD : 45056.8, kappa :512



	pcl::PointXYZ ph = frame.objects.at(objRef).getCentroid();


	pcl::PointXYZ headVector;
	headVector.x = frame.objects.at(obj).getCentroid().x - ph.x;
	headVector.y = frame.objects.at(obj).getCentroid().y - ph.y;
	headVector.z = 0; //ph.z - 100*head_ori[5];

	float meanH = -190.618;
	float sigmaH =58.1901;

	float meanD = 91128.7;
	float sigmaD = 45056.8;

	float kappa = 150; // 512;




	for(size_t i = 0; i < samples.points.size(); i++){
		// for every point find distance
		//float dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		float dist = ph.z - samples.points.at(i).z;
		// compute score
		double p1 = exp(-pow(((dist-meanH)/sigmaH),2)/2) /(sigmaH*2.50599);
		p1 = p1*pow(10,5);
		pcl::PointXYZ objVector;
		objVector.x = samples.points.at(i).x - ph.x;
		objVector.y = samples.points.at(i).y - ph.y;
		float angle = acos((headVector.x*objVector.x + headVector.y*objVector.y)/( sqrt(headVector.x*headVector.x + headVector.y*headVector.y) *sqrt(objVector.x*objVector.x + objVector.y*objVector.y)));

		double p2 = getPdf_VM(angle,0,kappa);
		p2 = p2*pow(10,5);
		dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		double p3 = exp(-pow(((dist-meanD)/sigmaD),2)/2) /(sigmaD*2.50599) ;
		p3 = p3*pow(10,5);

		samples.points.at(i).score =  p1*p2*p3;
		//cout << "point " <<  i << " score 1: " << p1 << " score 2 : " << p2  << " score 3: " << p3 << " score : " << samples.points.at(i).score << endl;

		//cout << "point " <<  i << " score 1: " << p1 << " score 2 :" << p2  << endl;

	}

	// find the points with non-zero score and add to point cloud

	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}

void affordanceMaps::placeabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples ,int obj, pcl::PointXYZ refPoint){
	//Parameters: N: 32, meanH : -190.618 , sigmaH : 58.1901, meanD : 91128.7 , sigmaD : 45056.8, kappa :512


	pcl::PointXYZ ph = refPoint;


	pcl::PointXYZ headVector;
	headVector.x = frame.objects.at(obj).getCentroid().x - ph.x;
	headVector.y = frame.objects.at(obj).getCentroid().y - ph.y;
	headVector.z = 0; //ph.z - 100*head_ori[5];

	float meanH = -190.618;
	float sigmaH =58.1901;

	float meanD = 91128.7;
	float sigmaD = 45056.8;

	float kappa = 150; // 512;


	float default_score = 1/samples.points.size();

	for(size_t i = 0; i < samples.points.size(); i++){
		// for every point find distance
		//float dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
	/*	float dist = ph.z - samples.points.at(i).z;
		// compute score
		double p1 = exp(-pow(((dist-meanH)/sigmaH),2)/2) /(sigmaH*2.50599);
		p1 = p1*pow(10,5);
		pcl::PointXYZ objVector;
		objVector.x = samples.points.at(i).x - ph.x;
		objVector.y = samples.points.at(i).y - ph.y;
		float angle = acos((headVector.x*objVector.x + headVector.y*objVector.y)/( sqrt(headVector.x*headVector.x + headVector.y*headVector.y) *sqrt(objVector.x*objVector.x + objVector.y*objVector.y)));

		double p2 = getPdf_VM(angle,0,kappa);
		p2 = p2*pow(10,5);
		dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		double p3 = exp(-pow(((dist-meanD)/sigmaD),2)/2) /(sigmaD*2.50599) ;
		p3 = p3*pow(10,5);
*/
		samples.points.at(i).score =  default_score;
		//cout << "point " <<  i << " score 1: " << p1 << " score 2 : " << p2  << " score 3: " << p3 << " score : " << samples.points.at(i).score << endl;

		//cout << "point " <<  i << " score 1: " << p1 << " score 2 :" << p2  << endl;

	}

	// find the points with non-zero score and add to point cloud

	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}


void affordanceMaps::cleanabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples ,int obj, int objRef ){
	//Parameters: N: 32, meanH : -190.618 , sigmaH : 58.1901, meanD : 91128.7 , sigmaD : 45056.8, kappa :512



	pcl::PointXYZ ph = frame.objects.at(objRef).getCentroid();


	pcl::PointXYZ headVector;
	headVector.x = frame.objects.at(obj).getCentroid().x - ph.x;
	headVector.y = frame.objects.at(obj).getCentroid().y - ph.y;
	headVector.z = 0; //ph.z - 100*head_ori[5];

	float meanH = -190.618;
	float sigmaH =58.1901;

	float meanD = 91128.7;
	float sigmaD = 45056.8;

	float kappa = 150; // 512;




	for(size_t i = 0; i < samples.points.size(); i++){
		// for every point find distance
		//float dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		float dist = ph.z - samples.points.at(i).z;
		// compute score
		double p1 = exp(-pow(((dist-meanH)/sigmaH),2)/2) /(sigmaH*2.50599);
		p1 = p1*pow(10,5);
		pcl::PointXYZ objVector;
		objVector.x = samples.points.at(i).x - ph.x;
		objVector.y = samples.points.at(i).y - ph.y;
		float angle = acos((headVector.x*objVector.x + headVector.y*objVector.y)/( sqrt(headVector.x*headVector.x + headVector.y*headVector.y) *sqrt(objVector.x*objVector.x + objVector.y*objVector.y)));

		double p2 = getPdf_VM(angle,0,kappa);
		p2 = p2*pow(10,5);
		dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		double p3 = exp(-pow(((dist-meanD)/sigmaD),2)/2) /(sigmaD*2.50599) ;
		p3 = p3*pow(10,5);

		samples.points.at(i).score =  p1*p2*p3;
		//cout << "point " <<  i << " score 1: " << p1 << " score 2 : " << p2  << " score 3: " << p3 << " score : " << samples.points.at(i).score << endl;

		//cout << "point " <<  i << " score 1: " << p1 << " score 2 :" << p2  << endl;

	}

	// find the points with non-zero score and add to point cloud

	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}

void affordanceMaps::drinkabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples  ){
	//Parameters: N: 75, meanH : 102.556 , sigmaH : 60.5061, meanD : 51974.1 , sigmaD : 24387, kappa :10
	//Parameters: N: 75, meanH : 102.556 , sigmaH : 60.5061, meanD : 51974.1 , sigmaD : 24387, kappa :512


	pcl::PointXYZ ph = frame.skeleton.transformed_joints.at(0);

	pcl::PointXYZ headVector;
	headVector.x = frame.skeleton.headOrientation.x - ph.x;
	headVector.y = frame.skeleton.headOrientation.y - ph.y;
	headVector.z = 0; //ph.z - 100*head_ori[5];

	float meanH = 102.556;
	float sigmaH = 60.5061;

	float meanD = 51974.1;
	float sigmaD = 24387;

	float kappa = 15;




	for(size_t i = 0; i < samples.points.size(); i++){
		// for every point find distance
		//float dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		float dist = ph.z - samples.points.at(i).z;
		// compute score
		double p1 = exp(-pow(((dist-meanH)/sigmaH),2)/2) /(sigmaH*2.50599);
		p1 = p1*pow(10,5);
		pcl::PointXYZ objVector;
		objVector.x = samples.points.at(i).x - ph.x;
		objVector.y = samples.points.at(i).y - ph.y;
		float angle = acos((headVector.x*objVector.x + headVector.y*objVector.y)/( sqrt(headVector.x*headVector.x + headVector.y*headVector.y) *sqrt(objVector.x*objVector.x + objVector.y*objVector.y)));

		double p2 = getPdf_VM(angle,0,kappa);
		p2 = p2*pow(10,5);
		dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		double p3 = exp(-pow(((dist-meanD)/sigmaD),2)/2) /(sigmaD*2.50599) ;
		p3 = p3*pow(10,5);

		samples.points.at(i).score =  p1*p2*p3;
		//cout << "point " <<  i << " score 1: " << p1 << " score 2 :" << p2  << endl;

	}

	// find the points with non-zero score and add to point cloud

	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}

void affordanceMaps::closeabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples , int objId ){
	int numJoints = frame.skeleton.transformed_joints.size();
	int joint = numJoints -1;
	float ldist = getMinDistance(frame,numJoints-1,objId);
	float rightdist = getMinDistance(frame,numJoints-2,objId);
	if(rightdist<ldist){joint = numJoints -2;}
	pcl::PointXYZ ph = frame.skeleton.transformed_joints.at(joint);

	float meanH = 0;
	float sigmaH = 10;

	float meanD = getDistanceSqrBwPoints(ph,frame.objects.at(objId).getCentroid());
	float sigmaD = 100;


	for(size_t i = 0; i < samples.points.size(); i++){
		// for every point find distance
		//float dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		float dist = ph.z - samples.points.at(i).z;
		// compute score
		double p1 = exp(-pow(((dist-meanH)/sigmaH),2)/2) /(sigmaH*2.50599);
		p1 = p1*pow(10,5);

		dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		double p3 = exp(-pow(((dist-meanD)/sigmaD),2)/2) /(sigmaD*2.50599) ;
		p3 = p3*pow(10,5);

		samples.points.at(i).score =  p1*p3;
		//cout << "point " <<  i << " score 1: " << p1 << " score 2 :" << p2  << endl;

	}

	// find the points with non-zero score and add to point cloud

	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}

void affordanceMaps::openabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples , int objId, int joint ){

	pcl::PointXYZ ph = frame.skeleton.transformed_joints.at(joint);
	pcl::PointXYZ pr = frame.objects.at(objId).getCentroid();

	pcl::PointXYZ headVector;
	headVector.x =  ph.x - pr.x;
	headVector.y =  ph.y - pr.y;
	headVector.z = 0; //ph.z - 100*head_ori[5];

	float meanH = 0;
	float sigmaH = 10;

	float meanD = getDistanceSqrBwPoints(ph,pr);
	float sigmaD = 100;

	float kappa = 15;


	for(size_t i = 0; i < samples.points.size(); i++){
		// for every point find distance
		//float dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		float dist = ph.z - samples.points.at(i).z;
		// compute score
		double p1 = exp(-pow(((dist-meanH)/sigmaH),2)/2) /(sigmaH*2.50599);
		p1 = p1*pow(10,5);
		pcl::PointXYZ objVector;
		objVector.x = samples.points.at(i).x - pr.x;
		objVector.y = samples.points.at(i).y - pr.y;
		float angle = acos((headVector.x*objVector.x + headVector.y*objVector.y)/( sqrt(headVector.x*headVector.x + headVector.y*headVector.y) *sqrt(objVector.x*objVector.x + objVector.y*objVector.y)));

		double p2 = getPdf_VM(angle,0,kappa);
		p2 = p2*pow(10,5);
		dist = getDistanceSqrBwPoints(ph,samples.points.at(i));
		double p3 = exp(-pow(((dist-meanD)/sigmaD),2)/2) /(sigmaD*2.50599) ;
		p3 = p3*pow(10,5);

		samples.points.at(i).score =  p1*p2*p3;
		//cout << "point " <<  i << " score 1: " << p1 << " score 2 :" << p2  << endl;

	}

	// find the points with non-zero score and add to point cloud

	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}




void affordanceMaps::getObjectPoints(Frame &f, int objid, pcl::PointCloud<pcl::PointXYZRGBScore> &cloud ){
	int numPoints = f.objects.at(objid).pcInds.size();
	cloud.width    = numPoints ;
	cloud.height   = 1;
	cloud.is_dense = false;
	//cloud.points.resize (cloud.width * cloud.height);
	for(int i = 0; i< numPoints; i++){
		int index = f.objects.at(objid).pcInds.at(i);
		pcl::PointXYZRGBScore np;
		np.x = f.cloud.at(index).x;
		np.y = f.cloud.at(index).y;
		np.z = f.cloud.at(index).z;
		np.rgb = f.cloud.at(index).rgb;
		np.score = 0;
		cloud.points.push_back(np);
	}
}

void affordanceMaps::writeHeatMap(string filename, vector<pcl::PointXYZRGBScore> &imageScorePoints){
	std::ofstream outFile;
	outFile.open(filename.c_str()); //, ios::app);
	vector<float> scores (Y_RES*X_RES,0);
	for (size_t i = 0 ; i < imageScorePoints.size(); i++){
		int x = imageScorePoints.at(i).x;
		int y = imageScorePoints.at(i).y;
		int index = ( x) + ( y) * X_RES;
		//cout << "x :" << x << " y: "<< y << " index: " << index << endl;
		if(index >0  && index < Y_RES*X_RES &&  scores.at(index)<  imageScorePoints.at(i).score ){
		scores.at(index) = imageScorePoints.at(i).score ;
		}
	}
	for (int index = 0; index < Y_RES*X_RES; index++){

			outFile << scores.at(index) << endl;

	}
	outFile.close();
}

vector<pcl::PointXYZRGBScore> affordanceMaps::generateSamples(pcl::PointCloud<pcl::PointXYZRGBScore> &cloud){
	vector<pcl::PointXYZRGBScore> points;
	vector<double> scores;
	boost::mt19937 gen;
	for (size_t i = 0; i < cloud.points.size(); i ++){
		scores.push_back(cloud.points.at(i).score);
	}
	boost::random::discrete_distribution<> dist(scores.begin(), scores.end());
	for (int i = 0; i < 3; i ++){
		int index = dist(gen);
		cout << "sampled point " << i <<  " "  << cloud.points.at(index).x << "," << cloud.points.at(index).y << "," << cloud.points.at(index).z << "," << cloud.points.at(index).score << endl;
		points.push_back(cloud.points.at(index));
	}
	return points;
}

vector<pcl::PointXYZRGBScore> affordanceMaps::generateReachSamples(pcl::PointCloud<pcl::PointXYZRGBScore> &cloud){
	vector<pcl::PointXYZRGBScore> points;
	vector<double> scores;
	boost::mt19937 gen;
	for (size_t i = 0; i < cloud.points.size(); i ++){
		scores.push_back(cloud.points.at(i).score);
	}
	boost::random::discrete_distribution<> dist(scores.begin(), scores.end());
	for (int i = 0; i < 5; i ++){
		int index = dist(gen);
		cout << "sampled point " << i <<  " "  << cloud.points.at(index).x << "," << cloud.points.at(index).y << "," << cloud.points.at(index).z << "," << cloud.points.at(index).score << endl;
		points.push_back(cloud.points.at(index));
	}
	return points;
}









vector<pcl::PointXYZRGBScore> affordanceMaps::generateDrinkabilityHeatMap(Frame &f, int s, int u ){


		pcl::PointCloud<pcl::PointXYZRGBScore> outCloud;
		pcl::copyPointCloud(f.cloud,outCloud);
		cout << "made copy:" << outCloud.points.size() << endl;
		//pair<float, float> handProb = findActiveHand(segment,q);
        //cout << "seg: " << s << " frame: "<< q <<" right: " << handProb.first << " left: " << handProb.second << endl;

		pcl::PointCloud<pcl::PointXYZRGBScore> samplePoints;
		getSamplePoints(f.skeleton.transformed_joints.at(0), 600, 20, samplePoints);

		drinkabilityScores(f,s,0, samplePoints);
		outCloud  += samplePoints;
		//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		//string filename = "heatmap_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".txt";
		//string imagefilename = "image_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".png";
		//saveImage(f.cloud,imagefilename);

		//vector<pcl::PointXYZRGBScore> imageScorePoints;
		//for(size_t i = 0; i < samplePoints.points.size(); i ++){
		//	 imageScorePoints.push_back(getImagePoint(samplePoints.points.at(i), globalTransform));
		//}
		//writeHeatMap(filename,imageScorePoints);
		return generateSamples(samplePoints);
}

vector<pcl::PointXYZRGBScore> affordanceMaps::generateOpenabilityHeatMap(Frame &f, int s, int u , int openableObj){

		int numJoints = f.skeleton.transformed_joints.size();
		int joint = numJoints -1;
		float ldist = getMinDistance(f,numJoints-1,openableObj);
		float rightdist = getMinDistance(f,numJoints-2,openableObj);
		if(rightdist<ldist){joint = numJoints -2;}

		pcl::PointCloud<pcl::PointXYZRGBScore> outCloud;
		pcl::copyPointCloud(f.cloud,outCloud);
		cout << "made copy:" << outCloud.points.size() << endl;
		//pair<float, float> handProb = findActiveHand(segment,q);
        //cout << "seg: " << s << " frame: "<< q <<" right: " << handProb.first << " left: " << handProb.second << endl;

		pcl::PointCloud<pcl::PointXYZRGBScore> samplePoints;
		getSamplePoints(f.skeleton.transformed_joints.at(joint), 600, 20, samplePoints);

		openabilityScores(f,s,0, samplePoints, openableObj, joint);
		outCloud  += samplePoints;
		//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		/*string filename = "heatmap_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".txt";
		string imagefilename = "image_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".png";
		saveImage(f.cloud,imagefilename);

		vector<pcl::PointXYZRGBScore> imageScorePoints;
		for(size_t i = 0; i < samplePoints.points.size(); i ++){
			 imageScorePoints.push_back(getImagePoint(samplePoints.points.at(i), globalTransform));
		}
		writeHeatMap(filename,imageScorePoints);*/
		return generateSamples(samplePoints);
}

vector<pcl::PointXYZRGBScore> affordanceMaps::generateCloseabilityHeatMap(Frame &f, int s, int u, int closeableObj ){


		pcl::PointCloud<pcl::PointXYZRGBScore> outCloud;
		pcl::copyPointCloud(f.cloud,outCloud);
		cout << "made copy:" << outCloud.points.size() << endl;
		//pair<float, float> handProb = findActiveHand(segment,q);
        //cout << "seg: " << s << " frame: "<< q <<" right: " << handProb.first << " left: " << handProb.second << endl;

		pcl::PointCloud<pcl::PointXYZRGBScore> samplePoints;
		getObjectPoints(f,closeableObj, samplePoints);

		closeabilityScores(f,s,0, samplePoints, closeableObj);
		outCloud  += samplePoints;
		//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		/*string filename = "heatmap_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".txt";
		string imagefilename = "image_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".png";
		saveImage(f.cloud,imagefilename);

		vector<pcl::PointXYZRGBScore> imageScorePoints;
		for(size_t i = 0; i < samplePoints.points.size(); i ++){
			 imageScorePoints.push_back(getImagePoint(samplePoints.points.at(i), globalTransform));
		}
		writeHeatMap(filename,imageScorePoints);*/
		return generateSamples(samplePoints);
}



void affordanceMaps::reachabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &outcloud, int handJoint){

	for (size_t i = 0; i < outcloud.points.size(); i ++){
		outcloud.points.at(i).score = 0;
	}

	int numjoints = frame.skeleton.transformed_joints.size();

    // head location
    pcl::PointXYZ ph = frame.skeleton.transformed_joints.at(0);

    pcl::PointXYZ headVector;
    headVector.x = frame.skeleton.headOrientation.x - ph.x;
    headVector.y = frame.skeleton.headOrientation.y - ph.y;
    headVector.z = 0; //ph.z - 100*head_ori[5];
    pair <float,int> pm = getMinObjDistance(frame,handJoint);
    float mean = pm.first;
    if (mean > 100) {mean = 100;}
    float sigma = 100;

	for( int i = 0; i < frame.objects.size(); i ++){

		for(int j = 0; j < frame.objects.at(i).pcInds.size(); j++){
			int index = frame.objects.at(i).pcInds.at(j);
			pcl::PointXYZ objVector;
			objVector.x = frame.cloud.at(index).x - ph.x;
			objVector.y = frame.cloud.at(index).y - ph.y;
			//cout << "head: (" << headVector.x  << "," << headVector.y << ") obj: (" << objVector.x << "," << objVector.y << ")" << endl;
			// find angle with head orientation
			float angle = acos((headVector.x*objVector.x + headVector.y*objVector.y)/( sqrt(headVector.x*headVector.x + headVector.y*headVector.y) *sqrt(objVector.x*objVector.x + objVector.y*objVector.y)));

			double p = getPdf_VM(angle,0,1);
			// distance to hand
			double dist = sqrt(getDistanceSqrBwPoints(frame.cloud.at(index),frame.skeleton.transformed_joints.at(handJoint)));
			double p1 = exp(-pow(((dist-mean)/sigma),2)/2) /(sigma*2.50599);


			if(j == 1){
			  cout << s << "," << q << "," << i <<" angle" << angle << " p:" << p << endl;
			}
			outcloud.points.at(index).score +=  p;
		}
	}
	pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outcloud);
}


vector<pcl::PointXYZRGBScore> affordanceMaps::generateReachabilityHeatMap(Frame &f, int s, int u, int handJoint ){


		pcl::PointCloud<pcl::PointXYZRGBScore> outCloud;
		pcl::copyPointCloud(f.cloud,outCloud);
		cout << "made copy:" << outCloud.points.size() << endl;
		//pair<float, float> handProb = findActiveHand(segment,q);
        //cout << "seg: " << s << " frame: "<< q <<" right: " << handProb.first << " left: " << handProb.second << endl;

		//pcl::PointCloud<pcl::PointXYZRGBScore> samplePoints;
		//getSamplePoints(f.skeleton.transformed_joints.at(0), 600, 20, samplePoints);

		reachabilityScores(f,s,0, outCloud,handJoint);
		////outCloud  += samplePoints;
		//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
		//string filename = "heatmap_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".txt";
		//string imagefilename = "image_"+ f.sequenceId + boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(u)+".png";
		//saveImage(f.cloud,imagefilename);

		//vector<pcl::PointXYZRGBScore> imageScorePoints;
		//for(size_t i = 0; i < samplePoints.points.size(); i ++){
		//	 imageScorePoints.push_back(getImagePoint(samplePoints.points.at(i), globalTransform));
		//}
		//writeHeatMap(filename,imageScorePoints);
		return generateReachSamples(outCloud);
}


vector<pcl::PointXYZRGBScore> affordanceMaps::generatePourabilityHeatMap(
		Frame &f, int s, int u, int pourableObj,
		int pourtoObj) {

	pcl::PointCloud < pcl::PointXYZRGBScore > samplePoints;

	pcl::PointCloud < pcl::PointXYZRGBScore > outCloud;
	pcl::copyPointCloud(f.cloud, outCloud);
	cout << "made copy:" << outCloud.points.size() << endl;
	getSamplePoints(f.objects.at(pourtoObj).getCentroid(), 600, 20, samplePoints);

	pourabilityScores(f, s, 0, samplePoints, pourableObj, pourtoObj);
	outCloud += samplePoints;
	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" +  boost::lexical_cast<std::string>(pour) + "_" + boost::lexical_cast<std::string>(pto)+".pcd", outCloud);
	//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
	/*string filename = "heatmap_" + boost::lexical_cast < std::string
			> (s) + "_" + boost::lexical_cast < std::string
			> (pourableObj) + "_" + boost::lexical_cast < std::string
			> (u) + "_" + boost::lexical_cast < std::string
			> (pourtoObj) + ".txt";
	string imagefilename = "pouring_" + boost::lexical_cast < std::string
			> (s) + "_" + boost::lexical_cast < std::string
			> (u) + "_" + boost::lexical_cast < std::string
			> (pourableObj) + "_" + boost::lexical_cast < std::string
			> (pourtoObj) + ".png";
	//saveImage(f.cloud,imagefilename);

	vector < pcl::PointXYZRGBScore > imageScorePoints;
	for (size_t i = 0; i < samplePoints.points.size(); i++) {
		imageScorePoints.push_back(
				getImagePoint(samplePoints.points.at(i), globalTransform));
	}
	writeHeatMap(filename, imageScorePoints);*/
	vector < pcl::PointXYZRGBScore > points = generateSamples(samplePoints);
//	vector < pcl::PointXYZRGBScore > imgPoints;
//	for (int i = 0; i < points.size(); i++) {
//		imgPoints.push_back(getImagePoint(points.at(i), globalTransform));
//	}
//	saveImage(f.cloud, imgPoints, imagefilename);
	return points;

}

vector<pcl::PointXYZRGBScore> affordanceMaps::generatePlaceabilityHeatMap(
		Frame &f, int s, int u, int placeableObj, pcl::PointXYZ refLoc ) {

	pcl::PointCloud < pcl::PointXYZRGBScore > samplePoints;

	pcl::PointCloud < pcl::PointXYZRGBScore > outCloud;
	pcl::copyPointCloud(f.cloud, outCloud);
	cout << "made copy:" << outCloud.points.size() << endl;
	getPlacingSamplePoints(f, 300, 800, samplePoints, placeableObj);

	placeabilityScores(f, s, 0, samplePoints,placeableObj, refLoc);
	outCloud += samplePoints;
	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" +  boost::lexical_cast<std::string>(pour) + "_" + boost::lexical_cast<std::string>(pto)+".pcd", outCloud);
	//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
	string filename = "heatmap_" + boost::lexical_cast < std::string
			> (s) + "_" + boost::lexical_cast < std::string
			> (placeableObj) + ".txt";
	string imagefilename = "pouring_" + boost::lexical_cast < std::string
			> (s) + "_" + boost::lexical_cast < std::string
			> (u) + "_" + boost::lexical_cast < std::string
			> (placeableObj) + ".txt";

	//saveImage(f.cloud,imagefilename);

	vector < pcl::PointXYZRGBScore > imageScorePoints;
//	for (size_t i = 0; i < samplePoints.points.size(); i++) {
//		imageScorePoints.push_back(
//				getImagePoint(samplePoints.points.at(i), globalTransform));
//	}
//	writeHeatMap(filename, imageScorePoints);
	vector < pcl::PointXYZRGBScore > points = generateSamples(samplePoints);
//	vector < pcl::PointXYZRGBScore > imgPoints;
//	for (int i = 0; i < points.size(); i++) {
//		imgPoints.push_back(getImagePoint(points.at(i), globalTransform));
//	}
//	saveImage(f.cloud, imgPoints, imagefilename);
	return points;

}


vector<pcl::PointXYZRGBScore> affordanceMaps::generateCleanabilityHeatMap(Frame &f, int s, int u, int cleanerObj,int cleanableObj) {

	pcl::PointCloud < pcl::PointXYZRGBScore > samplePoints;

	pcl::PointCloud < pcl::PointXYZRGBScore > outCloud;
	pcl::copyPointCloud(f.cloud, outCloud);
	cout << "made copy:" << outCloud.points.size() << endl;
	getObjectPoints(f, cleanableObj, samplePoints);

	cleanabilityScores(f, s, 0, samplePoints, cleanerObj, cleanableObj);
	outCloud += samplePoints;
	//pcl::io::savePCDFileASCII ("cloud_"+  boost::lexical_cast<std::string>(s) +"_" +  boost::lexical_cast<std::string>(pour) + "_" + boost::lexical_cast<std::string>(pto)+".pcd", outCloud);
	//pcl::io::savePCDFileASCII ("samplescloud_"+  boost::lexical_cast<std::string>(s) +"_" + boost::lexical_cast<std::string>(q)+".pcd", outCloud);
	/*string filename = "heatmap_" + boost::lexical_cast < std::string
			> (s) + "_" + boost::lexical_cast < std::string
			> (cleanerObj) + "_" + boost::lexical_cast < std::string
			> (u) + "_" + boost::lexical_cast < std::string
			> (cleanableObj) + ".txt";
	string imagefilename = "pouring_" + boost::lexical_cast < std::string
			> (s) + "_" + boost::lexical_cast < std::string
			> (u) + "_" + boost::lexical_cast < std::string
			> (cleanerObj) + "_" + boost::lexical_cast < std::string
			> (cleanableObj) + ".png";
	//saveImage(f.cloud,imagefilename);

	vector < pcl::PointXYZRGBScore > imageScorePoints;
	for (size_t i = 0; i < samplePoints.points.size(); i++) {
		imageScorePoints.push_back(
				getImagePoint(samplePoints.points.at(i), globalTransform));
	}
	writeHeatMap(filename, imageScorePoints);*/
	vector < pcl::PointXYZRGBScore > points = generateSamples(samplePoints);
//	vector < pcl::PointXYZRGBScore > imgPoints;
//	for (int i = 0; i < points.size(); i++) {
//		imgPoints.push_back(getImagePoint(points.at(i), globalTransform));
//	}
//	saveImage(f.cloud, imgPoints, imagefilename);
	return points;

}


pair <float,int> affordanceMaps::getMinObjDistance(Frame &frame, int joint){

	float minDist = 1000000;
	int objIdx = -1;
	for( int i = 0; i < frame.objects.size(); i ++){

			for(int j = 0; j < frame.objects.at(i).pcInds.size(); j++){
				int index = frame.objects.at(i).pcInds.at(j);
				float dist = sqrt(getDistanceSqrBwPoints(frame.cloud.at(index),frame.skeleton.transformed_joints.at(joint)));
				if(dist<minDist){minDist = dist; objIdx = i;}
			}
	}
	return make_pair(minDist, objIdx);
}

float affordanceMaps::getMinDistance(Frame &frame, int joint, int objId){

	float minDist = 1000000;



	for(int j = 0; j < frame.objects.at(objId).pcInds.size(); j++){
		int index = frame.objects.at(objId).pcInds.at(j);
		float dist = sqrt(getDistanceSqrBwPoints(frame.cloud.at(index),frame.skeleton.transformed_joints.at(joint)));
		if(dist<minDist){minDist = dist; }
	}

	return minDist;
}


void affordanceMaps::readAffMap(){
	string filename =  "affordanceMap.txt";

	ifstream file((char*) filename.c_str(), ifstream::in);
	map<string, int> SegCount;
	string line;
	int count = 0;
	while (getline(file, line)) {
		stringstream lineStream(line);
	    string element1, element2;
	    getline(lineStream, element1, ',');
	    while(getline(lineStream, element2, ',')){
	    	AffMap[element1].insert(element2);
	    }
	}
}


std::set<string> affordanceMaps::getAffordanceList(string label){
	std::set<string> affset;
	/*if(label.compare("cup") == 0) {
		affset.insert("drinkable");
		affset.insert("pourable");
		affset.insert("placeable");
	}*/
	cout << "affset for label: "<< label << " :";
	for (std::set<string>::iterator it=AffMap[label].begin(); it!=AffMap[label].end(); ++it)
	    std::cout << " ," << *it;
	cout << endl;
	return AffMap[label];
}

int affordanceMaps::getObjId(Frame &frame, PointXYZRGBScore &point){
	float minDist = 1000000;
	int objIdx = -1;
	if(frame.objects.size() ==1 ){return 0;}
	for( int i = 0; i < frame.objects.size(); i ++){

		for(int j = 0; j < frame.objects.at(i).pcInds.size(); j++){
			int index = frame.objects.at(i).pcInds.at(j);
			float dist = getDistanceSqrBwPoints(frame.cloud.at(index),point);
			if(dist<minDist){minDist = dist; objIdx = i;}
		}
	}
	return objIdx;
}

void affordanceMaps::generateHallucinations(vector<Frame> &segment, map< int, vector < hallucination > > &hallucinations, int segNum, int segLen, int numframeseen, int updateC){
	//vector < pair < pair <int, string> , pair <pcl::PointXYZ, vector<PointXYZRGBScore> > > > hallucinations;
	int halLen = segLen-numframeseen;
 	float activeObjThreshold = 75;
	int frameIdx = segment.size()-1;
	int numJoints = segment.at(0).skeleton.transformed_joints.size();
	// find active object
	int leftHandJoint =  numJoints -2;
	int rightHandJoint = numJoints -1;
	pair <float , int> leftDist = getMinObjDistance(segment.at(frameIdx),leftHandJoint);
	pair <float , int> rightDist = getMinObjDistance(segment.at(frameIdx),rightHandJoint);


	// for all objects
	for(int j = 0; j < segment.at(frameIdx).objects.size(); j++){
	  // check if active
		cout << "object id : " << j << " " ;
		bool active = false;
		vector<int> activehands  ;
		if (leftDist.first < activeObjThreshold && leftDist.second == j ){
			active = true;
			activehands.push_back(leftHandJoint);
			cout << "left hand active ";
		}else if (rightDist.first < activeObjThreshold && rightDist.second ==j ) {
			active = true;
			activehands.push_back(rightHandJoint);
			cout << "right hand active " ;
		}else {
			// object cannot moving.. no trajectory generated
			cout << "not in contact " ;
		}
		if (active){

			//pcl::PointXYZ startPoint = segment.at(frameIdx).skeleton.transformed_joints.at(hand);
			// get affordance list based on object type:
			cout << "obj type: " << segment.at(frameIdx).objects.at(j).getObjectType() << " ";
			set<string> affset = getAffordanceList(segment.at(frameIdx).objects.at(j).getObjectType());
			float score = 0;
			//if(affset.size()!=0) {score = 1/affset.size();}
			// drinking
			if(affset.find("drinkable") != affset.end()){
				pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(j).getCentroid();
				cout << "object is drinkable" << endl;
				vector<pcl::PointXYZRGBScore> endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
				cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
				for (size_t k = 0; k < endPoints.size(); k++){
					// if distance to the current obj location is very small.. then drinking action
					if(sqrt(getDistanceSqrBwPoints(startPoint,endPoints.at(k))) < 200){
						vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
						string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
						hallucination h (j, "drinking", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
						//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
						if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
					}else{
					// else is moving to drink action
						vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
						string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
						hallucination h (j, "moving", "drinkable", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
						//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
						if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
					}
				}
			}
			// opening
			if(affset.find("openable") != affset.end() ){
				pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(j).getCentroid();
				cout << "object is openable" << endl;
				if( segment.at(frameIdx).objects.at(j).getObjectType().compare("medcinebox") == 0){
					pcl::PointXYZRGBScore endPoint;
					vector<PointXYZ> trajPoints = getStationaryTrajPoints(startPoint, halLen);
					string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
					hallucination h (j, "opening", "", startPoint, endPoint, segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
					//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
					if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
				}else{
					vector<pcl::PointXYZRGBScore> endPoints = generateOpenabilityHeatMap(segment.at(frameIdx),segNum,updateC,j);
					cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
					for (size_t k = 0; k < endPoints.size(); k++){
						vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
						string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
						hallucination h (j, "opening", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
						//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
						if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }

					}
				}
			}
			// closing
			if(affset.find("closeable") != affset.end()){
				pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(j).getCentroid();
				cout << "object is closeable" << endl;
				vector<pcl::PointXYZRGBScore> endPoints = generateCloseabilityHeatMap(segment.at(frameIdx),segNum,updateC, j);
				cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
				for (size_t k = 0; k < endPoints.size(); k++){
					vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
					string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
					hallucination h (j, "closing", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
					//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
					if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }

				}
			}
			// pouring
			if(affset.find("pourable") != affset.end()){
				pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(j).getCentroid();
				cout << "object is pourable" << endl;
				vector<int> pourtoObj;
				for (int i = 0; i < segment.at(frameIdx).objects.size(); i++) {
					if (segment.at(frameIdx).objects.at(i).getObjectType().compare("bowl") == 0) {
						pourtoObj.push_back(i);
					}
				}
				int pourableObj = j;
				for (size_t pourto = 0; pourto < pourtoObj.size(); pourto++) {
					vector<pcl::PointXYZRGBScore> endPoints = generatePourabilityHeatMap(segment.at(frameIdx),segNum,updateC, pourableObj, pourtoObj.at(pourto));
					set<int> interactingObj;
					interactingObj.insert(pourto);
					for (size_t k = 0; k < endPoints.size(); k++){
						if(sqrt(getDistanceSqrBwPoints(startPoint,endPoints.at(k))) < 200){
							vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
							string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
							hallucination h (j, "pouring", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands,interactingObj);
							//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
							if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
						}else{
							vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
							string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
							hallucination h (j, "moving", "pourable", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
							//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
							if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
						}
					}
				}
			}
			// placing
			if(affset.find("placeable") != affset.end()){
				pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(j).getCentroid();
				cout << "object is placeable" << endl;
				int placeableObj = j;
				int foldindex = frameIdx-2;
				if(foldindex < 0) {foldindex = 0;}
				vector<pcl::PointXYZRGBScore> endPoints = generatePlaceabilityHeatMap(segment.at(frameIdx),segNum,updateC,placeableObj, segment.at(foldindex).objects.at(placeableObj).getCentroid() );
				cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
				for (size_t k = 0; k < endPoints.size(); k++){
					// if distance to the current obj location is very small.. then drinking action
					if(sqrt(getDistanceSqrBwPoints(startPoint,endPoints.at(k))) < 200){
						vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
						string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
						hallucination h (j, "placing", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
						//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
						if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
					}else{
						// else is moving to place action
						vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
						string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
						hallucination h (j, "moving", "placeable", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
									//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
						if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
					}
				}
			}
			// cleaning
			if(affset.find("cleaner") != affset.end()){
				pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(j).getCentroid();
				cout << "object is a cleaner" << endl;
				vector<int> cleanableObj;
				for (int i = 0; i < segment.at(frameIdx).objects.size(); i++) {
					if (segment.at(frameIdx).objects.at(i).getObjectType().compare("microwave") == 0) {
						cleanableObj.push_back(i);
					}
				}
				int cleanerObj = j;
				for (size_t cleanable = 0; cleanable < cleanableObj.size(); cleanable++) {
					vector<pcl::PointXYZRGBScore> endPoints = generateCleanabilityHeatMap(segment.at(frameIdx),segNum,updateC, cleanerObj, cleanableObj.at(cleanable));
					set<int> interactingObj;
					interactingObj.insert(cleanableObj.at(cleanable));
					for (size_t k = 0; k < endPoints.size(); k++){
						if(sqrt(getDistanceSqrBwPoints(startPoint,endPoints.at(k))) < 200){
							vector<PointXYZ> trajPoints = getCleaningTrajPoints(startPoint, endPoints.at(k), halLen,segment.at(frameIdx),cleanableObj.at(cleanable));
							string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
							hallucination h (j, "cleaning", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands,interactingObj);
							//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
							if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); }
						}else{
							vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
							string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
							hallucination h (j, "moving", "cleaner", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, score, id,activehands);
							//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
							if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
						}
					}
				}
			}


		}
		cout << endl;
	}

	// for both hands
	if (leftDist.first >= activeObjThreshold){
		vector<pcl::PointXYZRGBScore> endPoints;
		float score = 0;
		cout << "left hand free " << endl;
		pcl::PointXYZ startPoint = segment.at(frameIdx).skeleton.transformed_joints.at(leftHandJoint);
	// if not active then generate one of the following motion:
		// null
		// eating
		cout << "generating end point for moving to eat with left hand" << endl;
		endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
		for (size_t k = 0; k < endPoints.size(); k++) {
			if(sqrt(getDistanceSqrBwPoints(startPoint,endPoints.at(k))) < 200){
				vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
				string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
				hallucination h(-2, "eating", "", startPoint, endPoints.at(k), segLen, trajPoints,
						segment.at(frameIdx), globalTransform,score,id);
				//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

				if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
			}else{
				vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
				string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
				hallucination h(-2, "moving", "eat", startPoint, endPoints.at(k), segLen, trajPoints,
						segment.at(frameIdx), globalTransform,score,id);
				//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

				if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
			}
		}
		// reaching
		endPoints = generateReachabilityHeatMap(segment.at(frameIdx),segNum,updateC,leftHandJoint);
		for (size_t k = 0; k < endPoints.size(); k++) {
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			// find the reaching obj
			set<int> interactingObj;
			interactingObj.insert(getObjId(segment.at(frameIdx),endPoints.at(k)));
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h(-2, "reaching", "", startPoint, endPoints.at(k), segLen, trajPoints,
					segment.at(frameIdx), globalTransform,score,id, interactingObj);
					//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
		}

	}
	if (rightDist.first >= activeObjThreshold){
		float score = 0;
		cout << "right hand free " << endl;
		pcl::PointXYZ startPoint = segment.at(frameIdx).skeleton.transformed_joints.at(rightHandJoint);
		// if not active then generate one of the following motion:
			// null
			// eating
		cout << "generating end point for moving to eat with right hand" << endl;
		vector<pcl::PointXYZRGBScore> endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
		for (size_t k = 0; k < endPoints.size(); k++) {
			if(sqrt(getDistanceSqrBwPoints(startPoint,endPoints.at(k))) < 200){
				vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
				string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
				hallucination h(-1, "eating", "", startPoint, endPoints.at(k), segLen, trajPoints,
						segment.at(frameIdx), globalTransform,score, id);
				//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

				if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
			}else{
				vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
				string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
				hallucination h(-1, "moving", "eat", startPoint, endPoints.at(k), segLen, trajPoints,
						segment.at(frameIdx), globalTransform,score, id);
				//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

				if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
			}
		}
		// reaching
		endPoints = generateReachabilityHeatMap(segment.at(frameIdx),segNum,updateC,rightHandJoint);
		for (size_t k = 0; k < endPoints.size(); k++) {
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			// find the reaching obj
			set<int> interactingObj;
			interactingObj.insert(getObjId(segment.at(frameIdx),endPoints.at(k)));
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h(-1, "reaching", "", startPoint, endPoints.at(k), segLen, trajPoints,
					segment.at(frameIdx), globalTransform,score,id,interactingObj);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h);}
		}
	}




	// returns a set of sampled points for each affordance
	// object id, affordance, list of start and end points (can be empty for stationary)

}


void affordanceMaps::updateH(vector<Frame> &segment,  hallucination  &hal, map< int, vector < hallucination > > &hallucinations, int segNum, int segLen, int numframeseen, int updateC,  map< pair < int ,pair< string, string> > , int> &particleCounts){
	int frameIdx = segment.size()-1;
	int halLen = segLen-numframeseen;
	int objId = hal.objectId;
	int numJoints = segment.at(0).skeleton.transformed_joints.size();
	pair < int , pair< string, string > > actaff = make_pair(objId,make_pair(hal.activity, hal.affordance));
	if(hal.activity.compare("drinking")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is drinkable" << endl;
		vector<pcl::PointXYZRGBScore> endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
		cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
		for (size_t k = 0; k < endPoints.size(); k++){

			vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "drinking", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

		}
	}

	if(hal.activity.compare("moving")==0 && hal.affordance.compare("drinkable")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is drinkable" << endl;
		vector<pcl::PointXYZRGBScore> endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
		cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
		for (size_t k = 0; k < endPoints.size(); k++){
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "moving", "drinkable", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

		}
	}

	if(hal.activity.compare("opening")==0 && particleCounts[actaff]<5 ){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is openable" << endl;
		if( segment.at(frameIdx).objects.at(objId).getObjectType().compare("medcinebox") == 0){
			pcl::PointXYZRGBScore endPoint;
			vector<PointXYZ> trajPoints = getStationaryTrajPoints(startPoint , halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "opening", "", startPoint, endPoint,segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}
		}
		else{
			vector<pcl::PointXYZRGBScore> endPoints = generateOpenabilityHeatMap(segment.at(frameIdx),segNum,updateC,objId);
			cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
			for (size_t k = 0; k < endPoints.size(); k++){
				vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
				string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
				hallucination h (objId, "opening", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
				//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
				if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

			}
		}
	}

	if(hal.activity.compare("closing")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is drinkable" << endl;
		vector<pcl::PointXYZRGBScore> endPoints = generateCloseabilityHeatMap(segment.at(frameIdx),segNum,updateC, objId);
		cout << "number of endpoints sampled: " <<  endPoints.size() << endl;
		for (size_t k = 0; k < endPoints.size(); k++){
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "closing", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

		}
	}

	if(hal.activity.compare("pouring")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is pourable" << endl;

		int pourableObj = objId;
		int pourto = *(hal.interactingObj.begin());

		vector<pcl::PointXYZRGBScore> endPoints = generatePourabilityHeatMap(segment.at(frameIdx),segNum,updateC, pourableObj, pourto);

		for (size_t k = 0; k < endPoints.size(); k++){

			vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "pouring", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands, hal.interactingObj);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}


		}
	}
	if(hal.activity.compare("moving")==0 && hal.affordance.compare("pourable")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is pourable" << endl;

		int pourableObj = objId;
		int pourto = *(hal.interactingObj.begin());

		vector<pcl::PointXYZRGBScore> endPoints = generatePourabilityHeatMap(segment.at(frameIdx),segNum,updateC, pourableObj, pourto);

		for (size_t k = 0; k < endPoints.size(); k++){

			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "moving", "pourable", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

		}
	}


	if(hal.activity.compare("placing")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is placeable" << endl;
		int placeableObj = objId;
		int foldindex = frameIdx-2;
		if(foldindex < 0) {foldindex = 0;}
		vector<pcl::PointXYZRGBScore> endPoints = generatePlaceabilityHeatMap(segment.at(frameIdx),segNum,updateC,placeableObj, segment.at(foldindex).objects.at(placeableObj).getCentroid() );

		for (size_t k = 0; k < endPoints.size(); k++){
			vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "placing", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

		}
	}

	if(hal.activity.compare("moving")==0 && hal.affordance.compare("placeable")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();
		cout << "object is placeable" << endl;
		int placeableObj = objId;
		int foldindex = frameIdx-2;
		if(foldindex < 0) {foldindex = 0;}

		vector<pcl::PointXYZRGBScore> endPoints = generatePlaceabilityHeatMap(segment.at(frameIdx),segNum,updateC,placeableObj, segment.at(foldindex).objects.at(placeableObj).getCentroid() );
		for (size_t k = 0; k < endPoints.size(); k++){

			// else is moving to place action
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "moving", "placeable", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}

		}
	}
	if(hal.activity.compare("cleaning")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();

		int cleanerObj = objId;
		int cleanable = *(hal.interactingObj.begin());

		vector<pcl::PointXYZRGBScore> endPoints = generateCleanabilityHeatMap(segment.at(frameIdx),segNum,updateC, cleanerObj, cleanable);

		for (size_t k = 0; k < endPoints.size(); k++){

			vector<PointXYZ> trajPoints = getCleaningTrajPoints(startPoint, endPoints.at(k), halLen,segment.at(frameIdx),cleanable);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "cleaning", "", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands,hal.interactingObj);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1; }

		}
	}
	if(hal.activity.compare("moving")==0 && hal.affordance.compare("cleaner")==0 && particleCounts[actaff]<5){
		pcl::PointXYZ startPoint = segment.at(frameIdx).objects.at(objId).getCentroid();

		int cleanerObj = objId;
		int cleanable = *(hal.interactingObj.begin());

		vector<pcl::PointXYZRGBScore> endPoints = generateCleanabilityHeatMap(segment.at(frameIdx),segNum,updateC, cleanerObj, cleanable);

		for (size_t k = 0; k < endPoints.size(); k++){

			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h (objId, "moving", "cleaner", startPoint, endPoints.at(k),segLen, trajPoints, segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.activeHands);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );
			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}
		}


	}
	if(hal.activity.compare("moving")==0 && hal.affordance.compare("eat")==0 && particleCounts[actaff]<5){
		int joint = numJoints + objId;
		pcl::PointXYZ startPoint = segment.at(frameIdx).skeleton.transformed_joints.at(joint);
		vector<pcl::PointXYZRGBScore> endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
		for (size_t k = 0; k < endPoints.size(); k++) {
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h(objId, "moving", "eat", startPoint, endPoints.at(k), segLen, trajPoints,
					segment.at(frameIdx), globalTransform, hal.matchdist, id);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}
		}

	}


	if(hal.activity.compare("eating")==0 && particleCounts[actaff]<5){
		int joint = numJoints + objId;
		pcl::PointXYZ startPoint = segment.at(frameIdx).skeleton.transformed_joints.at(joint);
		vector<pcl::PointXYZRGBScore> endPoints = generateDrinkabilityHeatMap(segment.at(frameIdx),segNum,updateC);
		for (size_t k = 0; k < endPoints.size(); k++) {
			vector<PointXYZ> trajPoints = getSLTrajPoints(startPoint, endPoints.at(k), halLen);
			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h(objId, "eating", "", startPoint, endPoints.at(k), segLen, trajPoints,
					segment.at(frameIdx), globalTransform, hal.matchdist, id);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}
		}

	}

	if(hal.activity.compare("reaching")==0 && particleCounts[actaff]<5){
		int joint = numJoints + objId;
		pcl::PointXYZ startPoint = segment.at(frameIdx).skeleton.transformed_joints.at(joint);
		vector<pcl::PointXYZRGBScore> endPoints = generateReachabilityHeatMap(segment.at(frameIdx),segNum,updateC,joint);
		for (size_t k = 0; k < endPoints.size(); k++) {
			vector<PointXYZ> trajPoints = getTrajPoints(startPoint, endPoints.at(k), halLen);

			string id =  boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(updateC) +"_" + boost::lexical_cast<std::string>(hallucinations[updateC].size());
			hallucination h(objId, "reaching", "", startPoint, endPoints.at(k), segLen, trajPoints,
					segment.at(frameIdx), globalTransform, hal.matchdist, id, hal.interactingObj);
			//h.setId( boost::lexical_cast<std::string>(segNum) +"_" + boost::lexical_cast<std::string>(hallucinations.size()) );

			if(h.isValidTrajectory()){ hallucinations[updateC].push_back(h); particleCounts[actaff] += 1;}
		}
	}

}




void affordanceMaps::updateHallucinations(vector<Frame> &segment, map< int, vector < hallucination > > &hallucinations, int segNum, int segLen,int numframeseen, int updateCount){
	//vector < pair < pair <int, string> , pair <pcl::PointXYZ, vector<PointXYZRGBScore> > > > hallucinations;
	// if no change in skeleton or object locations : no need to update

	map< pair < int ,  pair< string, string > > , int> particleCounts;

	// find match scores and update trajectories
	std::multimap<float, int> scoreset;
	for(size_t h = 0; h < hallucinations[updateCount-1].size(); h ++ ){
		pair < int , pair< string, string> > actaff = make_pair(hallucinations[updateCount-1].at(h).objectId ,make_pair(hallucinations[updateCount-1].at(h).activity, hallucinations[updateCount-1].at(h).affordance));
		if (particleCounts.find(actaff) == particleCounts.end()){particleCounts[actaff] = 0;}
		double matchScore = hallucinations[updateCount-1].at(h).findTrajectoryMatch(segment,numframeseen);
		if(matchScore>0.7){
			scoreset.insert(pair<float,int>(matchScore,h));
			cout << "HAL:" << hallucinations[updateCount-1].at(h).halId << " is good! matchScore is " << matchScore << " act: " << hallucinations[updateCount-1].at(h).objectId << "," << hallucinations[updateCount-1].at(h).activity << ","<< hallucinations[updateCount-1].at(h).affordance  << " pred score:" << hallucinations[updateCount-1].at(h).halscore << endl;
		}
		else {cout << "HAL:" << hallucinations[updateCount-1].at(h).halId << " is dropped! matchScore is " << matchScore<< " act: " << hallucinations[updateCount-1].at(h).objectId << "," << hallucinations[updateCount-1].at(h).activity << ","<< hallucinations[updateCount-1].at(h).affordance << " pred score:" << hallucinations[updateCount-1].at(h).halscore << endl; }
	}

	for (std::multimap<float,int>::reverse_iterator it=scoreset.rbegin(); it!=scoreset.rend(); ++it){

	//for(size_t h = 0; h < hallucinations[updateCount-1].size(); h ++ ){
		//if(hallucinations[updateCount-1].at(h).objectId >= 0){
//		double matchScore = hallucinations[updateCount-1].at(h).findTrajectoryMatch(segment,numframeseen);
//		if ( matchScore > 0.5) {
		int h = (*it).second;
		updateH(segment, hallucinations[updateCount-1].at(h),  hallucinations, segNum, segLen,  numframeseen, updateCount, particleCounts);
			//hallucinations[updateCount].push_back(hallucinations[updateCount-1].at(h));
	//	}
			//}

	}
	if(updateCount>=2){hallucinations.at(updateCount-2).clear();}
	// if none of the trajectories match .. possible error in skeleton tracking.. regenerate the hallucinations..
	if(hallucinations[updateCount].size()< 10){
		generateHallucinations(segment,hallucinations,segNum,segLen,numframeseen,updateCount);
	}
}

affordanceMaps::affordanceMaps(TransformG &gT){
	globalTransform = gT;
	readAffMap();
}

affordanceMaps::~affordanceMaps(){

}
