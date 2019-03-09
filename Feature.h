#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp> 
#include <opencv2/nonfree/nonfree.hpp> 
#include <QPolygon>
#include <vector>
#include "Line.h"

using namespace std;
using namespace cv;

class Feature
{
public:
	Feature(void);
	~Feature(void);
	void fast_feature(Mat const &image, vector<KeyPoint> &keyPoints, vector<Line* > &Line_mesh_);

private:

};

#endif