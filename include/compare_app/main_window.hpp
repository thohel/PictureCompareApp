/**
 * @file /include/compare_app/main_window.hpp
 *
 * @brief Qt based gui for compare_app.
 *
 * @date November 2010
 **/
#ifndef compare_app_MAIN_WINDOW_H
#define compare_app_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <boost/atomic.hpp>
#include <QTimer>
#include <opencv2/opencv.hpp>

namespace compare_app {

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
    void on_button_set_ref_image_clicked(bool check);
    void on_button_set_test_image_clicked(bool check);
    void on_button_folder_select_3_clicked(bool check);
    void updateView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    cv::VideoCapture cap;
    QTimer *timer;
    cv::Mat reference;
    cv::Mat test;
    boost::atomic_bool let_pic_rest;
    int rest_counter;
    cv::Mat process_image_set(cv::Mat before, cv::Mat after);
    cv::Mat restore_picture(int number, bool reference, bool final);
    cv::Mat get_image();
    void save_picture(cv::Mat picture, int number, bool reference, bool final);
    void start_cam_capture();
    QString get_picture_url(int number, bool reference, bool final);
    void swap_image(cv::Mat image);
    int test_img_counter;
};

}  // namespace compare_app

#endif // compare_app_MAIN_WINDOW_H
