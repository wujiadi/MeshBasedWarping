#include "Feature.h"

Feature::Feature()
{

}

Feature::~Feature()
{

}

void Feature::fast_feature(Mat const &image, vector<KeyPoint> &keyPoints, vector<Line* > &Line_mesh_)
{
	FastFeatureDetector fast(30); 
	fast.detect(image,keyPoints);  
	//creat_mesh(keyPoints, Line_mesh_);
}