/**
 * @file /include/qgrnd/main_window.hpp
 *
 * @brief Qt based gui for qgrnd.
 *
 * @date November 2010
 **/
#ifndef qgrnd_MAIN_WINDOW_H
#define qgrnd_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "qgrnd/main_window.hpp"
#include "opencv2/opencv.hpp"
#include <QImage>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qgrnd {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
	void showImage(const cv::Mat &src);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_rdhmptBtn_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateImageView();
    void updateEstRltView();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	cv::Mat videoFrame;
	QImage *qimg;
};

}  // namespace qgrnd

#endif // qgrnd_MAIN_WINDOW_H
