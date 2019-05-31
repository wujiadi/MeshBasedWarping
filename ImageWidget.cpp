#include "ImageWidget.h"
#include "ImageWrap.h"
#include "Feature.h"
#include "RBF.h"
#include "IDW.h"
#include <QImage>
#include <QPainter>
#include <QtWidgets> 
#include <iostream>

using std::cout;
using std::endl;

ImageWidget::ImageWidget(void)
{

	draw_status_ = false;
	show_mesh_ = true;

	Run_mode_ = kDefault;
	func = NULL;
	ET = NULL;

	Start_Point_.clear();
	End_Point_.clear();
}


ImageWidget::~ImageWidget(void)
{
	for (size_t i = 0; i < Line_array_.size(); i++)
	{
		if (Line_array_[i])
		{
			delete Line_array_[i];
			Line_array_[i] = NULL;
		}
	}
	Line_array_.clear();

	for (size_t i = 0; i < Line_mesh_.size(); i++)
	{
		if (Line_mesh_[i])
		{
			delete Line_mesh_[i];
			Line_mesh_[i] = NULL;
		}
	}

	for (size_t i = 0; i < Line_test_.size(); i++)
	{
		if (Line_test_[i])
		{
			delete Line_test_[i];
			Line_test_[i] = NULL;
		}
	}

	Line_mesh_.clear();
	Line_test_.clear();

	if (ET)
	{
		delete [] ET;
		func = NULL;
	}

	if (func)
	{
		delete func;
		func = NULL;
	}
}

void ImageWidget::mousePressEvent(QMouseEvent *event)
{
	if (Run_mode_ == KSetpoint)
	{
		if (Qt::LeftButton == event->button())
		{
			draw_status_ = true;

			start_point_ = end_point_ = event->pos();
		}
	}
}


void ImageWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (draw_status_)
	{
		end_point_ = event->pos();
	}
}

void ImageWidget::mouseReleaseEvent(QMouseEvent *event)
{
	int i;

	int imagewidth = image_mat_.cols;
	int imageheight = image_mat_.rows;

	Line* current_line_ = NULL;
	current_line_ = new Line(start_point_.rx(), start_point_.ry(), end_point_.rx(), end_point_.ry());
	Line_array_.push_back(current_line_);

	Start_Point_ << QPoint(start_point_.rx() - (width()-imagewidth)/2, start_point_.ry() - (height()-imageheight)/2);
	End_Point_ << QPoint(end_point_.rx() - (width()-imagewidth)/2, end_point_.ry() - (height()-imageheight)/2);

	draw_status_ = false;
}

void ImageWidget::paintEvent(QPaintEvent *paintevent)
{
	QPainter painter;
	painter.begin(this);

	// Draw background
	painter.setBrush(Qt::lightGray);
	QRect back_rect(0, 0, width(), height());
	painter.drawRect(back_rect);

	// Draw image
	QImage image_show = QImage( (unsigned char *)(image_mat_.data), image_mat_.cols, image_mat_.rows, image_mat_.step, QImage::Format_RGB888 );
	QRect rect = QRect(0, 0, image_show.width(), image_show.height());
	painter.drawImage(rect, image_show);
	// Draw line
	if (Run_mode_ == KSetpoint)
	{	
		QPen pen;
		pen.setWidth(4);
		pen.setBrush(Qt::red);
		painter.setPen(pen);

		for (size_t i = 0; i < Line_array_.size(); i++)
		{
			Line_array_[i]->Draw(painter);
		}
		painter.drawLine(start_point_, end_point_);
	}

	if (show_mesh_ == true)
	{
		QPen pen;
		pen.setWidth(1);
		pen.setBrush(Qt::green);
		painter.setPen(pen);

		for (size_t i = 0; i < Line_mesh_.size(); i++)
		{
			Line_mesh_[i]->Draw(painter);
		}

		pen.setBrush(Qt::red);
		painter.setPen(pen);

		for (size_t i = 0; i < Line_test_.size(); i++)
		{
			Line_test_[i]->Draw(painter);
		}
	}
	

	painter.end();

	update();


}

void ImageWidget::Open()
{
	// Open file
	QString fileName = QFileDialog::getOpenFileName(this, tr("Read Image"), ".", tr("Images(*.bmp *.png *.jpg)"));

	// Load file
	if (!fileName.isEmpty())
	{
		image_mat_ = cv::imread( fileName.toLatin1().data() );
		cvtColor( image_mat_, image_mat_, CV_BGR2RGB );	
		image_mat_backup_ = image_mat_.clone();
	}

	Feature feature_;
	feature_.fast_feature(image_mat_, keyPoints, Line_mesh_, ET, PQPoints);

	update();
}

void ImageWidget::Save()
{
	SaveAs();
}

void ImageWidget::SaveAs()
{
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Image"), ".", tr("Images(*.bmp *.png *.jpg)"));
	if (filename.isNull())
	{
		return;
	}	

	Mat image_save;
	cvtColor(image_mat_, image_save, CV_RGB2BGR);
	imwrite(filename.toLatin1().data(), image_save);
}

void ImageWidget::Invert()
{
	MatIterator_<Vec3b> iter, iterend;
	for (iter=image_mat_.begin<Vec3b>(), iterend=image_mat_.end<Vec3b>(); iter != iterend; ++iter)
	{
		(*iter)[0] = 255-(*iter)[0];
		(*iter)[1] = 255-(*iter)[1];
		(*iter)[2] = 255-(*iter)[2];
	}

	update();
}

void ImageWidget::Mirror(bool ishorizontal, bool isvertical)
{
	int width = image_mat_.cols;
	int height = image_mat_.rows;

	if (ishorizontal)
	{
		if (isvertical)
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					image_mat_.at<Vec3b>(j, i) = image_mat_backup_.at<Vec3b>(height-1-j, width-1-i);
				}
			}
		} 
		else
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					image_mat_.at<Vec3b>(j, i) = image_mat_backup_.at<Vec3b>(j, width-1-i);
				}
			}
		}

	}
	else
	{
		if (isvertical)
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					image_mat_.at<Vec3b>(j, i) = image_mat_backup_.at<Vec3b>(height-1-j, i);
				}
			}
		}
	}

	update();
}

void ImageWidget::TurnGray()
{
	MatIterator_<Vec3b> iter, iterend;
	for (iter=image_mat_.begin<Vec3b>(), iterend=image_mat_.end<Vec3b>(); iter != iterend; ++iter)
	{
		int itmp = ((*iter)[0]+(*iter)[1]+(*iter)[2])/3;
		(*iter)[0] = itmp;
		(*iter)[1] = itmp;
		(*iter)[2] = itmp;
	}

	update();
}

void ImageWidget::Restore()
{
	image_mat_ = image_mat_backup_.clone();
	update();
}

void ImageWidget::ImageWarp()
{
	switch (Run_mode_)
	{
	case kRBF:
		func = new RBF;
		func->Init(Start_Point_,End_Point_);									//get parameter
		func->DoWrapPoints(image_mat_, image_mat_backup_, Start_Point_, End_Point_, PQPoints, ET);   //do image wrap
		//func->DoWrapMesh(image_mat_, image_mat_backup_, PQPoints, ET);
		func->DoWrapTest(image_mat_, image_mat_backup_, PQPoints, ET, Line_mesh_, Line_test_);
		show_mesh_ = true;
		break;

	case kIDW:
		func = new IDW;
		func->Init(Start_Point_,End_Point_);									//get parameter
		func->DoWrapPoints(image_mat_, image_mat_backup_, Start_Point_, End_Point_, PQPoints, ET);   //do image wrap
		//func->DoWrapMesh(image_mat_, image_mat_backup_, PQPoints, ET);
		func->DoWrapTest(image_mat_, image_mat_backup_, PQPoints, ET, Line_mesh_, Line_test_);
		show_mesh_ = true;
		break;

	case KFix:
		func->FixImage(image_mat_,SEARCH_R, SEARCH_POINT);
		delete func;
		func = NULL;
		break;

	default:
		return;
		break;

	}

	update();

	return;

}

void ImageWidget::SetModeToIDW()
{
	if (Run_mode_ == KSetpoint)
	{
		Run_mode_ = kIDW;

		
		for (size_t i = 0; i < Line_array_.size(); i++)
		{
			if (Line_array_[i])
			{
				delete Line_array_[i];
				Line_array_[i] = NULL;
			}
		}

		Line_array_.clear();

		ImageWarp();
	}

}

void ImageWidget::SetModeToRBF()
{
	if (Run_mode_ == KSetpoint)
	{
		Run_mode_ = kRBF;

		
		for (size_t i = 0; i < Line_array_.size(); i++)
		{
			if (Line_array_[i])
			{
				delete Line_array_[i];
				Line_array_[i] = NULL;
			}
		}
		
		

		Line_array_.clear();

		ImageWarp();
	}

}

void ImageWidget::SetModeToSetPoint()
{

	if (Run_mode_ == KFix)
	{
		return;
	}
	Run_mode_ = KSetpoint;

	show_mesh_ = false;

	for (size_t i = 0; i < Line_mesh_.size(); i++)
	{
		if (Line_mesh_[i])
		{
			delete Line_mesh_[i];
			Line_mesh_[i] = NULL;
		}
	}

	for (size_t i = 0; i < Line_test_.size(); i++)
	{
		if (Line_test_[i])
		{
			delete Line_test_[i];
			Line_test_[i] = NULL;
		}
	}

	Line_mesh_.clear();
	Line_test_.clear();

	/*
	Start_Point_ << QPoint(0, 0);
	End_Point_ << QPoint(0, 0);

	Start_Point_ << QPoint(ptr_image_->width(), 0);
	End_Point_ << QPoint(ptr_image_->width(), 0);

	Start_Point_ << QPoint(ptr_image_->width(), ptr_image_->height());
	End_Point_ << QPoint(ptr_image_->width(), ptr_image_->height());

	Start_Point_ << QPoint(0, ptr_image_->height());
	End_Point_ << QPoint(0, ptr_image_->height());
	*/
	

}

void ImageWidget::SetModeToFix()
{
	if ((Run_mode_ == kIDW) || (Run_mode_ == kRBF))
	{
		Run_mode_ = KFix;

		ImageWarp();
	}

}
