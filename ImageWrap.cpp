#include "ImageWrap.h"
#include <cmath>
#include <QImage>
#include <ANN/ANN.h>					// ANN declarations

int				k				= 1;			// number of nearest neighbors
int				dim				= 2;			// dimension
double			eps				= 0;			// error bound
int				maxPts			= 1000;			// maximum number of data points
int             sqRad           = 1;

int overlapcount = 0;

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

int Wrap::DoWrapPoints(Mat &image, Mat &tempimage, QPolygon &StartPoints, QPolygon &EndPoints, vector<PQPoint> &PQPoints, list<edge>* ET)
{
	return 0;
}

int Wrap::DoWrap(Mat &image, QPolygon &StartPoints, QPolygon &EndPoints)
{
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

int Wrap::DoWrapMesh(Mat &image, Mat &orgimage, vector<PQPoint> &PQPoints, list<edge>* ET)
{

	list<edge> AET;

	int width = orgimage.cols;
	int height = orgimage.rows;

	//for mesh points
	for (int i = 0; i < height; i++)
	{
		//删除AET中满足y=ymax的边
		for(list<edge>::iterator aetit = AET.begin(); aetit != AET.end();)
		{
			if (aetit->ymax == i)
			{
				aetit = AET.erase(aetit);
			}
			else
			{
				aetit++;
			}
		}

		//取出ET中当前扫描行的所有边并按x的递增顺序（若x相等则按dx的递增顺序）插入AET
		for(list<edge>::iterator etit = ET[i].begin(); etit != ET[i].end(); etit++)
		{

			list<edge>::iterator aetit = AET.begin();
			while (aetit != AET.end())
			{
				if (etit->x > aetit->x)
				{
					aetit++;
					continue;
				}

				if ((etit->x == aetit->x) && (etit->dx > aetit->dx))
				{
					aetit++;
					continue;
				}

				break;

			}

			AET.insert (aetit,*etit); 

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
				BCinterpolation(edgeA, edgeB, i, image, orgimage, PQPoints);
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

int Wrap::DoWrapTest(Mat &image, Mat &orgimage, vector<PQPoint> &PQPoints, list<edge>* ET, vector<Line* > &Line_mesh_, vector<Line* > &Line_test_)
{

	int width = orgimage.cols;
	int height = orgimage.rows;

	Line_mesh_.clear();
	Line_test_.clear();

	//for mesh points
	for (int i = 0; i < height; i++)
	{
		//取出ET中当前扫描行的所有边并按x的递增顺序（若x相等则按dx的递增顺序）插入AET
		for(list<edge>::iterator etit = ET[i].begin(); etit != ET[i].end(); etit++)
		{
			Line* current_line_ = NULL;
			current_line_ = new Line(PQPoints[etit->PonitA].QPoint.x(), PQPoints[etit->PonitA].QPoint.y(), PQPoints[etit->PonitB].QPoint.x(), PQPoints[etit->PonitB].QPoint.y());
			Line_test_.push_back(current_line_);

			current_line_ = NULL;
			current_line_ = new Line(PQPoints[etit->PonitA].PPoint.x(), PQPoints[etit->PonitA].PPoint.y(), PQPoints[etit->PonitB].PPoint.x(), PQPoints[etit->PonitB].PPoint.y());
			Line_mesh_.push_back(current_line_);
		}
	}

	return 0;
}

int Wrap::BCinterpolation(edge &edgeA, edge &edgeB, int y, Mat &image, Mat &orgimage, vector<PQPoint> &PQPoints)
{
	int width = orgimage.cols;
	int height = orgimage.rows;
	int start,end = 0;

	QPoint VertexP[3];
	QPoint VertexQ[3];

	double Para[3];

	GetTrianglePonit(edgeA, edgeB, PQPoints, VertexP, VertexQ);

	start = (int)edgeA.x;
	end = (int)edgeB.x;
	if (start != edgeA.x)
	{
		start++;
	}

	for (int i = start; i <= end; i++)
	{
		QPoint temppoint, resultpoint;
		double coordinateX,coordinateY;

		temppoint.rx() = i;
		temppoint.ry() = y;

		Vec3i bgr;
		bgr = orgimage.at<Vec3b>(temppoint.ry(), temppoint.rx());

		GetBarycentricCoordinate(VertexP, temppoint, Para);

		coordinateX = Para[0] * VertexQ[0].x() + Para[1] * VertexQ[1].x() + Para[2] * VertexQ[2].x();
		coordinateY = Para[0] * VertexQ[0].y() + Para[1] * VertexQ[1].y() + Para[2] * VertexQ[2].y();

		resultpoint.rx() = coordinateX + 0.5;
		resultpoint.ry() = coordinateY + 0.5;

		if((resultpoint.x() >= 0) && (resultpoint.x() < width) && (resultpoint.y() >= 0) && (resultpoint.y() < height))
		{
			
			image.at<Vec3b>(resultpoint.y(), resultpoint.x()) = bgr;
			MatrixSet(resultpoint.y(), resultpoint.x()) = 1;
			
		}
		else
		{
			//printf("error point:\n");
		}

	}

	//printf("overlapcount %d:\n", overlapcount);


	return 0;

}

int Wrap::GetTrianglePonit(edge &edgeA, edge &edgeB, vector<PQPoint> &PQPoints, QPoint *VertexP, QPoint *VertexQ)
{
	int PonitA, PointB, PointC;

	PonitA = edgeA.PonitA;
	PointB = edgeA.PonitB;

	if ((edgeB.PonitA == PonitA)||(edgeB.PonitA == PointB))
	{
		PointC = edgeB.PonitB;
	}
	else if ((edgeB.PonitB == PonitA)||(edgeB.PonitB == PointB))
	{
		PointC = edgeB.PonitA;
	}
	else
	{
		return 1;
	}

	VertexP[0] = PQPoints[PonitA].PPoint;
	VertexP[1] = PQPoints[PointB].PPoint;
	VertexP[2] = PQPoints[PointC].PPoint;

	VertexQ[0] = PQPoints[PonitA].QPoint;
	VertexQ[1] = PQPoints[PointB].QPoint;
	VertexQ[2] = PQPoints[PointC].QPoint;

	return 0;

}

int Wrap::GetBarycentricCoordinate(QPoint *VertexP, QPoint x, double *Para)
{
	QPoint A,B,C;

	A = VertexP[0];
	B = VertexP[1];
	C = VertexP[2];

	Para[0]=(B.y()*C.x()*1.0f-B.x()*C.y()-B.y()*x.x()+C.y()*x.x()+B.x()*x.y()-C.x()*x.y())/(A.y()*B.x()*1.0f-A.x()*B.y()-A.y()*C.x()+B.y()*C.x()+A.x()*C.y()-B.x()*C.y());
	Para[1]=(-A.y()*C.x()*1.0f+A.x()*C.y()+A.y()*x.x()-C.y()*x.x()-A.x()*x.y()+C.x()*x.y())/(A.y()*B.x()*1.0f-A.x()*B.y()-A.y()*C.x()+B.y()*C.x()+A.x()*C.y()-B.x()*C.y());
	Para[2]=(A.y()*B.x()*1.0f-A.x()*B.y()-A.y()*x.x()+B.y()*x.x()+A.x()*x.y()-B.x()*x.y())/(A.y()*B.x()*1.0f-A.x()*B.y()-A.y()*C.x()+B.y()*C.x()+A.x()*C.y()-B.x()*C.y());

	return 0;
}


