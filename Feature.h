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
#include "ImageWrap.h"
#include "ImageWidget.h"
#include <stdio.h>
#include <stdlib.h>

extern "C"
{
#include "triangle.h"
}

using namespace std;
using namespace cv;

class Feature
{
public:
	Feature(void);
	~Feature(void);
	void fast_feature(Mat const &image, vector<KeyPoint> &keyPoints, vector<Line* > &Line_mesh_, list<edge>* &ET, vector<PQPoint>  &PQPoints);

private:
	void creat_mesh(Mat const &image, vector<KeyPoint> &keyPoints, vector<Line* > &Line_mesh_, list<edge>* &ET, vector<PQPoint> &PQPoints);
	void report(struct triangulateio * io, int markers, int reporttriangles, int reportneighbors, int reportsegments,int reportedges, int reportnorms);
	void AddEdgeToET(int startindex, int x1, int y1, int endindex, int x2, int y2, list<edge> * &ET);

};

#endif