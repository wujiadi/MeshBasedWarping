#include "ImageWrap.h"
#include <cmath>
#include <QImage>
#include <ANN/ANN.h>					// ANN declarations

int				k				= 1;			// number of nearest neighbors
int				dim				= 2;			// dimension
double			eps				= 0;			// error bound
int				maxPts			= 1000;			// maximum number of data points
int             sqRad           = 1;

Wrap::Wrap()
{

}

Wrap::~Wrap()
{

}
int Wrap::Init(QPolygon &StartPoints, QPolygon &EndPoints)
{
	return 0;
}

int Wrap::DoWrap(Mat &image, QPolygon &StartPoints, QPolygon &EndPoints, vector<PQPoint> &PQPoints, list<edge>* ET)
{
	QPoint temppoint;
	QPoint resultpoint;
	list<edge> AET;

	Mat tempimage = image.clone();
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

		if((resultpoint.x() > 0) && (resultpoint.x() < width) && (resultpoint.y() > 0) && (resultpoint.y() < height))
		{
			image.at<Vec3b>(resultpoint.y(), resultpoint.x()) = bgr;
			MatrixSet(resultpoint.y(), resultpoint.x()) = 1;

			PQPoints[i].QPoint.rx() = resultpoint.x();
			PQPoints[i].QPoint.ry() = resultpoint.y();
		}
	}

	//for mesh points
	for (int i = 0; i < height; i++)
	{
		//删除AET中满足y=ymax的边
		for(list<edge>::iterator aetit = AET.begin(); aetit != AET.end(); aetit++)
		{
			if (aetit->ymax == i)
			{
				AET.erase(aetit);
			}
		}

		//取出ET中当前扫描行的所有边并按x的递增顺序（若x相等则按dx的递增顺序）插入AET
		for(list<edge>::iterator etit = ET[i].begin(); etit != ET[i].end(); etit++)
		{
			for(list<edge>::iterator aetit = AET.begin(); aetit != AET.end(); aetit++)
			{
				if (etit->x > aetit->x)
				{
					continue;
				}

				if ((etit->x == aetit->x) && (etit->dx > aetit->dx))
				{
					continue;
				}

				AET.insert (aetit,etit); 
				break;
			}
		}

		//AET中的边两两配对并填色
		list<edge>::iterator aetit = AET.begin();
		while(1)
		{
			if (aetit == AET.end())
			{
				break;
			}
			edge edgeA = *aetit;

			aetit++;

			if (aetit == AET.end())
			{
				break;
			}
			edge edgeB = *aetit;

			if (edgeA.x == edgeB.x)
			{
				continue;
			}
			else
			{
				//重心坐标插值（edgeA.x,edgeB.x, i, image tempimage pq）
			}
		}

		//更新AET中边的x值，进入下一循环
		for(list<edge>::iterator aetit = AET.begin(); aetit != AET.end(); aetit++)
		{
			aetit->x += aetit->dx;
		}
	
	}

	return 0;
}

//searchradius:R^2
int Wrap::FixImage(Mat &image, int searchradius, int searchcount)
{
	int colorred,colorgreen,colorblue;
	int usecount;
	QRgb color;

	int width = image.cols;
	int height = image.rows;

	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	maxPts = height * width;
	k = searchcount;
	sqRad = searchradius;
	usecount = 0;

	colorred = colorblue = colorgreen = 0;

	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists


	nPts = 0;									// read data points

	cout << "Data Points:\n";
	for (int i=0; i<width; i++)
	{
		for (int j=0; j<height; j++)
		{
			if (MatrixSet(j,i) == 1)
			{
				dataPts[nPts][0] = i;			//2D
				dataPts[nPts][1] = j;
				nPts++;	
			}
		}
	}

	kdTree = new ANNkd_tree(					// build search structure
		dataPts,								// the data points
		nPts,									// number of points
		dim);									// dimension of space


	for (int i=0; i<width; i++)
	{
		for (int j=0; j<height; j++)
		{
			if (MatrixSet(j,i) == 0)
			{
				queryPt[0] = i;					//2D
				queryPt[1] = j;

				kdTree->annkFRSearch(			// search
					queryPt,					// query point
					sqRad,
					k,							// number of near neighbors
					nnIdx,						// nearest neighbors (returned)
					dists,						// distance (returned)
					eps);						// error bound

				colorred = 0;
				colorblue = 0;
				colorgreen = 0;

				for (usecount = 0; usecount < k; usecount++)
				{
					if (ANN_DIST_INF == dists[usecount])
					{
						break;
					}
					else
					{
						colorred = colorred + image.at<Vec3b>(dataPts[nnIdx[usecount]][1], dataPts[nnIdx[usecount]][0])[2];
						colorgreen = colorgreen + image.at<Vec3b>(dataPts[nnIdx[usecount]][1], dataPts[nnIdx[usecount]][0])[1];
						colorblue = colorblue + image.at<Vec3b>(dataPts[nnIdx[usecount]][1], dataPts[nnIdx[usecount]][0])[0];

					}

				}

				if(usecount != 0)
				{
					image.at<Vec3b>(j, i) = Vec3b(colorblue/usecount, colorgreen/usecount, colorred/usecount);
				}


			}
		}
	}

	delete [] nnIdx;							// clean things up
	delete [] dists;
	delete kdTree;
	annClose();									// done with ANN

	return 0;
}

double Wrap::GetDistance(QPoint PointA, QPoint PointB)
{
	double result;

	result = pow((double)(PointA.x() - PointB.x()), 2) + pow((double)(PointA.y() - PointB.y()), 2); //a^2 + b^2
	result = sqrt(result);                      

	return result;
}

