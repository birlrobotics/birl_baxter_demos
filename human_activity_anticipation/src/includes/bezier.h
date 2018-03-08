 int  factorial(int x, int result = 1) {
  if (x == 1 || x ==0) return result; else return factorial(x - 1, x * result);
}


 float Bernstein(int n, int j, float t){
	return factorial(n)/(factorial(j)* factorial(n-j))*pow(t,j)*pow((1-t),(n-j));
}

 vector<pcl::PointXYZ> Bezier(int numPoints, vector<pcl::PointXYZ> &controlPoints){
	vector<float> t;
	vector<pcl::PointXYZ> Q;
	for(int i = 0; i < numPoints ; i ++){
		t.push_back(i*1.0/(numPoints-1));
		//cout << t.at(i) << "," ;
	}
	for(size_t i = 0; i < t.size(); i ++){
		pcl::PointXYZ p(0,0,0);
		for(size_t j = 0; j < controlPoints.size(); j++){
			float val = Bernstein(controlPoints.size()-1, j, t.at(i));
			p.x = p.x+ controlPoints.at(j).x *val;
			p.y = p.y+ controlPoints.at(j).y *val;
			p.z = p.z+ controlPoints.at(j).z *val;
		}
		Q.push_back(p);
	}

	//cout << endl;
	return Q;

}
