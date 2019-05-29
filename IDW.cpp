#include "IDW.h"
#include <iostream>
#include <Dense>

using namespace std;
using namespace Eigen;

IDW::IDW()
{

}

IDW::~IDW()
{

}

int IDW::Init(QPolygon &StartPoints, QPolygon &EndPoints)
{
	GetCoeT(StartPoints, EndPoints);
	return 0;
}

int IDW::DoWrapPoints(Mat &image, Mat &tempimage, QPolygon &StartPoints, QPolygon &EndPoints, vector<PQPoint> &PQPoints, list<edge>* ET)
{
	QPoint temppoint;
	QPoint resultpoint;

	int width = image.cols;
	int height = image.rows;

	MatrixSet.resize(height, width);
	MatrixSet.setZero();

	for (int i=0; i<width; i++)
	{
		for (int j=0; j<height; j++)
		{
			image.at<Vec3b>(j, i) = Vec3b(255, 255, 255);
		}
	}

	//for P points
	int tempsize = PQPoints.size();
	for (int i = 0; i < tempsize; i++)
	{
		Vec3i bgr;
		bgr = tempimage.at<Vec3b>(PQPoints[i].PPoint.ry(), PQPoints[i].PPoint.rx());

		temppoint.rx() = PQPoints[i].PPoint.x();
		temppoint.ry() = PQPoints[i].PPoint.y();

		resultpoint = CalculatePixel(temppoint, StartPoints, EndPoints);
		PQPoints[i].QPoint.rx() = resultpoint.x();
		PQPoints[i].QPoint.ry() = resultpoint.y();

		if((resultpoint.x() >= 0) && (resultpoint.x() < width) && (resultpoint.y() >= 0) && (resultpoint.y() < height))
		{
			image.at<Vec3b>(resultpoint.y(), resultpoint.x()) = bgr;
			MatrixSet(resultpoint.y(), resultpoint.x()) = 1;
		}
	}

	return 0;
}


QPoint IDW::CalculatePixel(QPoint const &orgpoint, QPolygon &StartPoints, QPolygon &EndPoints)
{
	QPoint temppoint;
	int i,j;
	int MatrixSize = StartPoints.size();
	double diffx;
	double diffy;
	double point_x = 0;
	double point_y = 0;

	//need to check orgpoint

	GetCoeWeight(orgpoint, StartPoints);

	for (i = 0; i < MatrixSize; i++)
	{
		diffx = orgpoint.x() - StartPoints[i].x();
		diffy = orgpoint.y() - StartPoints[i].y();
		point_x += (weight[i] / Weightsum) * (EndPoints[i].x() + T(i, 0) * diffx + T(i, 1) * diffy);
		point_y += (weight[i] / Weightsum) * (EndPoints[i].y() + T(i, 2) * diffx + T(i, 3) * diffy);
	}

	temppoint.rx() = point_x;
	temppoint.ry() = point_y;

	return temppoint;
}

void IDW::GetCoeWeight(QPoint const &orgpoint, QPolygon &StartPoints)
{
	int i;
	double tempweight;
	int MatrixSize = StartPoints.size();


	Weightsum = 0;
	weight.clear();

	for (i = 0; i < MatrixSize; i++)
	{
		tempweight = pow(GetDistance(StartPoints[i], orgpoint), -2);
		weight.push_back(tempweight);
		Weightsum += tempweight;
	}

}

void IDW::GetCoeT(QPolygon &StartPoints, QPolygon &EndPoints)
{
	int i,j;
	int MatrixSize = StartPoints.size();
	double tempweight;

	MatrixXd A(2,2);
	VectorXd bx(2);
	VectorXd by(2);
	VectorXd X(2);
	VectorXd Y(2);

	T.resize(MatrixSize, 4);
	T.setZero();

	for (i = 0; i < MatrixSize; i++)
	{
		A.setZero();
		bx.setZero();
		by.setZero();
		X.setZero();
		Y.setZero();

		for (j = 0; j < MatrixSize; j++)
		{
			if (j != i)
			{
				tempweight = pow(GetDistance(StartPoints[i], StartPoints[j]), -2);
				A(0,0) += tempweight * (StartPoints[j].x() - StartPoints[i].x()) * (StartPoints[j].x() - StartPoints[i].x());
				A(0,1) += tempweight * (StartPoints[j].y() - StartPoints[i].y()) * (StartPoints[j].x() - StartPoints[i].x());
				A(1,0) += tempweight * (StartPoints[j].y() - StartPoints[i].y()) * (StartPoints[j].x() - StartPoints[i].x());
				A(1,1) += tempweight * (StartPoints[j].y() - StartPoints[i].y()) * (StartPoints[j].y() - StartPoints[i].y());

				bx(0) += tempweight * (EndPoints[j].x() - EndPoints[i].x()) * (StartPoints[j].x() - StartPoints[i].x());
				bx(1) += tempweight * (EndPoints[j].x() - EndPoints[i].x()) * (StartPoints[j].y() - StartPoints[i].y());

				by(0) += tempweight * (EndPoints[j].y() - EndPoints[i].y()) * (StartPoints[j].x() - StartPoints[i].x());
				by(1) += tempweight * (EndPoints[j].y() - EndPoints[i].y()) * (StartPoints[j].y() - StartPoints[i].y());
			}
		}

		X = A.colPivHouseholderQr().solve(bx);
		Y = A.colPivHouseholderQr().solve(by);

		T(i,0) = X(0);//get solution
		T(i,1) = X(1);
		T(i,2) = Y(0);
		T(i,3) = Y(1);

	}


}
