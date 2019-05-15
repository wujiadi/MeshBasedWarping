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

#include "ImageWidget.h"

using namespace cv;
using namespace std;
using namespace Eigen;


class Wrap
{
public:
	Wrap(void);
	virtual ~Wrap(void);
	virtual int Init(QPolygon &StartPoints, QPolygon &EndPoints);
	virtual QPoint CalculatePixel(QPoint const &orgpoint, QPolygon &StartPoints, QPolygon &EndPoints);

	int DoWrap(Mat &image, QPolygon &StartPoints, QPolygon &EndPoints, vector<PQPoint> &PQPoints, list<edge>* ET);
	int FixImage(Mat &image, int searchradius, int searchcount);                                       //Fix the image after wraping
	double GetDistance(QPoint PointA, QPoint PointB);

	MatrixXi MatrixSet;

private:
	int BCinterpolation(edge &edgeA, edge &edgeB, int y, Mat &image, Mat &orgimage, vector<PQPoint> &PQPoints);
	int GetTrianglePonit(edge &edgeA, edge &edgeB, vector<PQPoint> &PQPoints, QPoint *VertexP, QPoint *VertexQ);
	int Wrap::GetBarycentricCoordinate(QPoint *VertexP, QPoint x, double *Para);
};

#endif