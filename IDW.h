#ifndef IDW_H
#define IDW_H

#include <QPolygon>
#include <vector>
#include "ImageWrap.h"

using namespace std;

class IDW : public Wrap
{
public:
	IDW(void);
	~IDW(void);
	int Init(QPolygon &StartPoints, QPolygon &EndPoints);
	int DoWrapPoints(Mat &image, Mat &tempimage, QPolygon &StartPoints, QPolygon &EndPoints, vector<PQPoint> &PQPoints, list<edge>* ET);

private:
	void GetCoeT(QPolygon &StartPoints, QPolygon &EndPoints);
	void GetCoeWeight(QPoint const &orgpoint, QPolygon &StartPoints);
	QPoint CalculatePixel(QPoint const &orgpoint, QPolygon &StartPoints, QPolygon &EndPoints);

	MatrixXd T;
	vector<double> weight;
	double Weightsum;

};

#endif