#ifndef WRAP_H
#define WRAP_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <QPolygon>
#include <vector>
#include <list>
#include <QImage>
#include <iostream>
#include <Dense>

using namespace cv;
using namespace std;
using namespace Eigen;

struct edge
{
	int ymax;
	float x;
	float dx;
	int PonitA;
	int PonitB;
};

class Wrap
{
public:
	Wrap(void);
	virtual ~Wrap(void);
	virtual int Init(QPolygon &StartPoints, QPolygon &EndPoints);
	virtual int DoWrap(Mat &image, QPolygon &StartPoints, QPolygon &EndPoints);
	int FixImage(Mat &image, int searchradius, int searchcount);                                       //Fix the image after wraping
	double GetDistance(QPoint PointA, QPoint PointB);

	MatrixXi MatrixSet;

private:


};

#endif