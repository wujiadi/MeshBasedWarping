#ifndef RBF_H
#define RBF_H

#include <QPolygon>
#include <vector>
#include "ImageWrap.h"

using namespace std;

class RBF : public Wrap
{
public:
	RBF(void);
	~RBF(void);
	int Init(QPolygon &StartPoints, QPolygon &EndPoints);
	int DoWrapPoints(Mat &image, Mat &tempimage, QPolygon &StartPoints, QPolygon &EndPoints, vector<PQPoint> &PQPoints, list<edge>* ET);
	int DoWrap(Mat &image, QPolygon &StartPoints, QPolygon &EndPoints);

private:
	void GetCoeR(QPolygon &StartPoints);
	void GetCoeXY(QPolygon &StartPoints, QPolygon &EndPoints);
	QPoint CalculatePixel(QPoint const &orgpoint, QPolygon &StartPoints, QPolygon &EndPoints);

	vector<double> CoeX;
	vector<double> CoeY;	
	vector<double> CoeR;

};

#endif