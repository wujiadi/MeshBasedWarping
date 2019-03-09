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
	int DoWrap(Mat &image, QPolygon &StartPoints, QPolygon &EndPoints);

private:
	void GetCoeT(QPolygon &StartPoints, QPolygon &EndPoints);
	QPoint CalculatePixel(QPoint const &orgpoint, QPolygon &StartPoints, QPolygon &EndPoints);
	void GetCoeWeight(QPoint const &orgpoint, QPolygon &StartPoints);

	MatrixXd T;
	vector<double> weight;
	double Weightsum;

};

#endif