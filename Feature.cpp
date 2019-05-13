#include "Feature.h"

Feature::Feature()
{

}

Feature::~Feature()
{

}

//get feature points
void Feature::fast_feature(Mat const &image, vector<KeyPoint> &keyPoints, vector<Line* > &Line_mesh_, list<edge> * &ET, vector<PQPoint> &PQPoints)
{
	FastFeatureDetector fast(40);//30
	fast.detect(image,keyPoints);  
	creat_mesh(image, keyPoints, Line_mesh_, ET, PQPoints);
}

//creat mesh points according to the feature points
void Feature::creat_mesh(Mat const &image, vector<KeyPoint> &keyPoints, vector<Line* > &Line_mesh_, list<edge>* &ET, vector<PQPoint> &PQPoints)
{
	struct triangulateio in, mid, out, vorout;

	/* Define input points. */

	int width = image.cols;
	int height = image.rows;
	int keypointsize = keyPoints.size();

	ET = new list<edge>[height];

	in.numberofpoints = keyPoints.size() + 4;
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));

	in.pointlist[0] = 0;
	in.pointlist[1] = 0;
	in.pointlist[2] = 0;
	in.pointlist[3] = height;
	in.pointlist[4] = width;
	in.pointlist[5] = 0;
	in.pointlist[6] = width;
	in.pointlist[7] = height;


	for (int i = 0; i < keypointsize; i++)
	{
		in.pointlist[8 + i * 2] = keyPoints[i].pt.x;
		in.pointlist[8 + i * 2 + 1] = keyPoints[i].pt.y;
	}


	in.pointattributelist = (REAL *) NULL;

	in.pointmarkerlist = (int *) NULL;

	in.numberofsegments = 0;
	in.numberofholes = 0;
	in.numberofregions = 0;
	in.regionlist = (REAL *) NULL;

	printf("Input point set:\n\n");
	report(&in, 0, 0, 0, 0, 0, 0);



	/* Make necessary initializations so that Triangle can return a */
	/*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

	mid.pointlist = (REAL *) NULL;            /* Not needed if -N switch used. */
	/* Not needed if -N switch used or number of point attributes is zero: */
	mid.pointattributelist = (REAL *) NULL;
	mid.pointmarkerlist = (int *) NULL; /* Not needed if -N or -B switch used. */
	mid.trianglelist = (int *) NULL;          /* Not needed if -E switch used. */
	/* Not needed if -E switch used or number of triangle attributes is zero: */
	mid.triangleattributelist = (REAL *) NULL;
	mid.neighborlist = (int *) NULL;         /* Needed only if -n switch used. */
	/* Needed only if segments are output (-p or -c) and -P not used: */
	mid.segmentlist = (int *) NULL;
	/* Needed only if segments are output (-p or -c) and -P and -B not used: */
	mid.segmentmarkerlist = (int *) NULL;
	mid.edgelist = (int *) NULL;             /* Needed only if -e switch used. */
	mid.edgemarkerlist = (int *) NULL;   /* Needed if -e used and -B not used. */


	/* Triangulate the points.  Switches are chosen to read and write a  */
	/*   PSLG (p), preserve the convex hull (c), number everything from  */
	/*   zero (z), assign a regional attribute to each element (A), and  */
	/*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
	/*   neighbor list (n).                                              */

	triangulate("qczen", &in, &mid, &vorout);//pczen

	printf("Initial triangulation:\n\n");
	report(&mid, 0, 1, 1, 1, 1, 0);

	for (int i = 0; i < mid.numberofedges; i++) 
	{
		int startpoint, endpoint;

		startpoint = mid.edgelist[i * 2 + 0];
		endpoint = mid.edgelist[i * 2 + 1];

		Line* current_line_ = NULL;
		current_line_ = new Line((int)mid.pointlist[startpoint * 2], (int)mid.pointlist[startpoint * 2 + 1], (int)mid.pointlist[endpoint * 2], (int)mid.pointlist[endpoint * 2 + 1]);
		Line_mesh_.push_back(current_line_);

		AddEdgeToET(startpoint, (int)mid.pointlist[startpoint * 2], (int)mid.pointlist[startpoint * 2 + 1], endpoint, (int)mid.pointlist[endpoint * 2], (int)mid.pointlist[endpoint * 2 + 1], ET);

	}

	for (int i = 0; i < mid.numberofpoints; i++) 
	{
		PQPoint temppoint;
		temppoint.PPoint.rx() = (int)mid.pointlist[i * 2];
		temppoint.PPoint.ry() = (int)mid.pointlist[i * 2 + 1];

		PQPoints.push_back(temppoint);
	}

	/* Free all allocated arrays, including those allocated by Triangle. */

	free(in.pointlist);
	free(in.pointattributelist);
	free(in.pointmarkerlist);
	free(mid.pointlist);
	free(mid.pointattributelist);
	free(mid.pointmarkerlist);
	free(mid.trianglelist);
	free(mid.triangleattributelist);
	free(mid.neighborlist);
	free(mid.segmentlist);
	free(mid.segmentmarkerlist);
	free(mid.edgelist);
	free(mid.edgemarkerlist);

	return;
}

//just for debug
void Feature::report(struct triangulateio * io, int markers, int reporttriangles, int reportneighbors, int reportsegments,int reportedges, int reportnorms)
{
	int i, j;

	for (i = 0; i < io->numberofpoints; i++) {
		printf("Point %4d:", i);
		for (j = 0; j < 2; j++) {
			printf("  %.6g", io->pointlist[i * 2 + j]);
		}
		if (io->numberofpointattributes > 0) {
			printf("   attributes");
		}
		for (j = 0; j < io->numberofpointattributes; j++) {
			printf("  %.6g",
				io->pointattributelist[i * io->numberofpointattributes + j]);
		}
		if (markers) {
			printf("   marker %d\n", io->pointmarkerlist[i]);
		} else {
			printf("\n");
		}
	}
	printf("\n");

	if (reporttriangles || reportneighbors) {
		for (i = 0; i < io->numberoftriangles; i++) {
			if (reporttriangles) {
				printf("Triangle %4d points:", i);
				for (j = 0; j < io->numberofcorners; j++) {
					printf("  %4d", io->trianglelist[i * io->numberofcorners + j]);
				}
				if (io->numberoftriangleattributes > 0) {
					printf("   attributes");
				}
				for (j = 0; j < io->numberoftriangleattributes; j++) {
					printf("  %.6g", io->triangleattributelist[i *
						io->numberoftriangleattributes + j]);
				}
				printf("\n");
			}
			if (reportneighbors) {
				printf("Triangle %4d neighbors:", i);
				for (j = 0; j < 3; j++) {
					printf("  %4d", io->neighborlist[i * 3 + j]);
				}
				printf("\n");
			}
		}
		printf("\n");
	}

	if (reportsegments) {
		for (i = 0; i < io->numberofsegments; i++) {
			printf("Segment %4d points:", i);
			for (j = 0; j < 2; j++) {
				printf("  %4d", io->segmentlist[i * 2 + j]);
			}
			if (markers) {
				printf("   marker %d\n", io->segmentmarkerlist[i]);
			} else {
				printf("\n");
			}
		}
		printf("\n");
	}

	if (reportedges) {
		for (i = 0; i < io->numberofedges; i++) {
			printf("Edge %4d points:", i);
			for (j = 0; j < 2; j++) {
				printf("  %4d", io->edgelist[i * 2 + j]);
			}
			if (reportnorms && (io->edgelist[i * 2 + 1] == -1)) {
				for (j = 0; j < 2; j++) {
					printf("  %.6g", io->normlist[i * 2 + j]);
				}
			}
			if (markers) {
				printf("   marker %d\n", io->edgemarkerlist[i]);
			} else {
				printf("\n");
			}
		}
		printf("\n");
	}
}

//TODO
void Feature::AddEdgeToET(int startindex, int x1, int y1, int endindex, int x2, int y2, list<edge> * &ET)
{
	if (y1 == y2)
	{
		return;
	}

	int ymin = y1 > y2 ? y2 : y1;
	int ymax = y1 > y2 ? y1 : y2;
	float x = y1 > y2 ? x2 : x1;
	float dx = (x1 - x2) * 1.0f / (y1 - y2);

	edge tempedge;
	tempedge.PonitA = startindex;
	tempedge.PonitB = endindex;
	tempedge.x = x;
	tempedge.dx = dx;
	tempedge.ymax = ymax;

	ET[ymin].push_back(tempedge);

	return;
}