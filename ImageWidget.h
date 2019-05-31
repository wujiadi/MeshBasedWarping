#ifndef IMAGEW_H
#define IMAGEW_H

#include <QWidget>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp> 
#include <opencv2/nonfree/nonfree.hpp> 
#include <QPolygon>
#include <QMouseEvent>
#include <vector>
#include <list>
#include "Line.h"

using namespace std;
using namespace cv;

QT_BEGIN_NAMESPACE
class QImage;
class QPainter;
class QMouseEvent;
QT_END_NAMESPACE

enum RunMode
{
	kDefault = 0,
	KSetpoint = 1,
	kRBF = 2,
	kIDW = 3,
	KFix = 4,
};

struct edge
{
	int ymax;
	float x;
	float dx;
	int PonitA;
	int PonitB;
};

struct PQPoint
{
	QPoint PPoint;
	QPoint QPoint;
};

class Wrap;

#define SEARCH_R		4
#define SEARCH_POINT    4

class ImageWidget :
	public QWidget
{
	Q_OBJECT

public:
	ImageWidget(void);
	~ImageWidget(void);

protected:
	void paintEvent(QPaintEvent *paintevent);

public slots:
	// File IO
	void Open();												// Open an image file, support ".bmp, .png, .jpg" format
	void Save();												// Save image to current file
	void SaveAs();												// Save image to another file

	// Image processing
	void Invert();												// Invert pixel value in image
	void Mirror(bool horizontal=false, bool vertical=true);		// Mirror image vertically or horizontally
	void TurnGray();											// Turn image to gray-scale map
	void Restore();												// Restore image to origin

	void ImageWarp();                                           // Image wrap
	void SetModeToIDW();
	void SetModeToRBF();
	void SetModeToSetPoint();
	void SetModeToFix();

	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

private:
	Mat				 image_mat_;
	Mat				 image_mat_backup_;
	bool			 draw_status_;								 // draw line
	bool             show_mesh_;
	QPoint			 start_point_;
	QPoint			 end_point_;
	QPolygon         Start_Point_;                               //n pair control points
	QPolygon         End_Point_;                                 //n pair control points
	RunMode			 Run_mode_;

	vector<Line* >   Line_array_;
	vector<Line* >   Line_mesh_;
	vector<Line* >   Line_test_;

	vector<KeyPoint> keyPoints;
	vector<PQPoint>  PQPoints;

	list<edge>		 *ET;
	list<edge>       AET;

	Wrap			 *func;
};

#endif

