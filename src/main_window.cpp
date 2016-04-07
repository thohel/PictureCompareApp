/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/

#include <QtGui>
#include <iostream>
#include "../include/compare_app/main_window.hpp"
#include <opencv2/opencv.hpp>

#define CAM_FPS 2
#define TIMER_MS 10
#define MS_PAUSE 2000

namespace compare_app {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.button_comp_diff->setEnabled(false);
    ui.button_set_test_image->setEnabled(false);
    rest_counter = 0;
    test_img_counter = 0;
    let_pic_rest = false;

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateView()));
    timer->start(TIMER_MS);
}

MainWindow::~MainWindow() {}

static const QString type2str(int type) {
  QString r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

static QImage mat2qimage(cv::Mat& mat) {
    switch (mat.type()) {
        // 8-bit, 4 channel
        case CV_8UC4: {
            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB32);
            return image;
        }

        // 8-bit, 3 channel
        case CV_8UC3: {
            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
            return image.rgbSwapped();
        }

        // 8-bit, 1 channel
        case CV_8UC1: {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if (sColorTable.isEmpty()) {
                for (int i = 0; i < 256; ++i)
                    sColorTable.push_back(qRgb(i, i, i));
            }

            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
            image.setColorTable(sColorTable);

            return image;
        }

        default:
            std::cout << "cvMatToQImage() - cv::Mat image type not handled in switch: " << type2str(mat.type()).toUtf8().data();
            break;
    }

    return QImage();
}

void MainWindow::start_cam_capture()
{
    // Open the default camera, use something different from 0 otherwise
    if(!cap.open(0)) {
        std::cout << "Failed to open camera!" << std::endl;
        return;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 2304);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1536);
    cap.set(CV_CAP_PROP_FPS, CAM_FPS);
 }

cv::Mat MainWindow::process_image_set(cv::Mat before, cv::Mat after)
{
    cv::Mat final = after;
    cv::Mat temp;
    cv::Mat backup = after;

    /*
    if (ui->crossHairCheckBox->isChecked()) {
        cv::Point2i leftMid;
        leftMid.x = 0;
        leftMid.y = final.rows/2;
        cv::Point2i rightMid;
        rightMid.x = final.cols;
        rightMid.y = final.rows/2;
        cv::line(final, leftMid, rightMid, cv::Scalar(0, 255, 0), 2); // Crosshair horizontal
        cv::Point2i topMid;
        topMid.x = final.cols/2;
        topMid.y = 0;
        cv::Point2i botMid;
        botMid.x = final.cols/2;
        botMid.y = final.rows;
        cv::line(final, topMid, botMid, cv::Scalar(0, 255, 0), 2); // Crosshair vertical
    }

    // The picture has been set but the cropping sliders are not activated
    // We now know the size of the image, so we can adjust our sliders accordingly
    if (!ui->xposSlider->isEnabled()) {
        ui->xposSlider->setMinimum(0);
        ui->xposSlider->setMaximum(final.cols);
        ui->xposSlider->setEnabled(true);
        ui->heightSlider->setMinimum(0);
        ui->heightSlider->setMaximum(final.cols);
        ui->heightSlider->setEnabled(true);
        ui->yposSlider->setMinimum(0);
        ui->yposSlider->setMaximum(final.rows);
        ui->yposSlider->setEnabled(true);
        ui->widthSlider->setMinimum(0);
        ui->widthSlider->setMaximum(final.rows);
        ui->widthSlider->setEnabled(true);
    }
*/

//    if (ui->diffCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::absdiff(before, after, temp);
        final = temp;
//    }
/*
    cv::Rect roi;
    if (ui->cropingCheckBox->isChecked() || ui->cropingVisualizeCheckBox->isChecked()) {
        // Define a box that is our Region of Interest
        roi.x = ui->xposSlider->value();
        roi.y = ui->yposSlider->value();
        roi.width = (roi.x + ui->widthSlider->value() > final.cols) ? final.cols - roi.x : ui->widthSlider->value();
        roi.height = (roi.y + ui->heightSlider->value() > final.rows) ? final.rows - roi.y : ui->heightSlider->value();
    }

    if (ui->cropingCheckBox->isChecked()) {
        temp = final(roi);
        final = temp;
    } else if (ui->cropingVisualizeCheckBox->isChecked()) { // XXX: Should not be added to image when we start 3D capture
        cv::rectangle(final, roi, cv::Scalar(0, 255, 0));
    }

    if (ui->sharpeningCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::Mat temp2;
        cv::GaussianBlur(final, temp2, cv::Size(0, 0), 5);
        cv::addWeighted(final, 3, temp2, -1, 0, temp);
        final = temp;
    }

    if (ui->medianCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Post-processing to remove noise
        int kernelSize = ui->medianSlider->value();

        if (kernelSize % 2 != 1)
            kernelSize = kernelSize - 1;

        if (kernelSize < 1)
            kernelSize = 1;

        cv::medianBlur(final, temp, kernelSize);
        final = temp;
    }

    if (ui->erosionCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        int erosion_size = ui->erosionSlider->value();
        cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                cv::Point( erosion_size, erosion_size ) );
        // Apply the erosion operation
        erode(final, temp, element );
        final = temp;
    }

    if (ui->cannyCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Detect edges using canny
        cv::Canny(final, temp, ui->cannySlider->value(), 255, 3 , true);

        cv::vector<cv::vector<cv::Point> > contours;
        cv::vector<cv::Vec4i> hierarchy;

        // Find contours
        findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        // Draw contours
        final = cv::Mat::zeros(temp.size(), CV_8UC1 );
        for(uint i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(255);
            drawContours(final, contours, i, color, 2, -2, hierarchy, 2, cv::Point() );
        }
    }

    if (ui->cropingCheckBox->isChecked()) {
        final.copyTo(backup(roi));
        final = backup;
    }

    if (ui->checkBox->isChecked() && ui->mainCheckBox->isChecked() && !ui->brightnessCheckBox->isChecked()) {
        cv::inRange(final, cv::Scalar(ui->blueSliderMin->value(), ui->greenSliderMin->value(), ui->redSliderMin->value()),
                         cv::Scalar(ui->blueSliderMax->value(), ui->greenSliderMax->value(), ui->redSliderMax->value()), temp);
        final = temp;
    }
*/
    return final;
}

QString MainWindow::get_picture_url(int number, bool reference, bool final)
{
    QString url = ui.textbox_path_3->toPlainText();
    url.append("/");
    url.append(ui.fileName_3->text());
    url.append("_");
    url.append(QString("%1").arg(number, 3, 10, QChar('0')));

    if (final) {
        url.append("_final");
    } else if (reference) {
        url.append("_reference");
    } else {
        url.append("_test");
    }

    url.append(".png");
    return url;
}

void MainWindow::save_picture(cv::Mat picture, int number, bool reference, bool final)
{
    if (!ui.saveCheckBox_3->isChecked())
        return;

    QString url = get_picture_url(number, reference, final);

    cv::imwrite(url.toUtf8().constData(), picture);
}

cv::Mat MainWindow::restore_picture(int number, bool reference, bool final)
{
    QString url = get_picture_url(number, reference, final);

    return cv::imread(url.toUtf8().constData());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::on_button_set_ref_image_clicked(bool check)
{
    ui.button_set_test_image->setEnabled(true);
    let_pic_rest = true;
    cv::Mat img = get_image();
    reference = img.clone();
    save_picture(img, 0, true, false);
    swap_image(img);
}

void MainWindow::on_button_set_test_image_clicked(bool check)
{
    ui.button_comp_diff->setEnabled(true);
    let_pic_rest = true;
    cv::Mat img = get_image();
    test = img.clone();
    save_picture(img, test_img_counter, false, false);
    swap_image(img);
    test_img_counter++;
}

void MainWindow::on_button_comp_diff_clicked(bool check)
{
    let_pic_rest = true;
    cv::Mat img = process_image_set(reference, test);
    save_picture(img, test_img_counter, false, true);
    swap_image(img);
}

void MainWindow::on_button_folder_select_3_clicked(bool check)
{
    QString fileName;
    fileName = QFileDialog::getExistingDirectory(this, tr("Choose Or Create Directory"), "/home/minions", QFileDialog::DontResolveSymlinks);
    ui.textbox_path_3->setText(fileName);
    ui.saveCheckBox_3->setEnabled(true);
}

cv::Mat MainWindow::get_image()
{
    cv::Mat image;

    cap >> image;

    assert(!image.empty() && image.type() == CV_8UC3);

    return image;
}

void MainWindow::swap_image(cv::Mat image)
{
    std::cout << "Converting a new frame" << std::endl;
    QImage toShow = mat2qimage(image);
    QPixmap pixMap = QPixmap::fromImage(toShow);
    std::cout << "Showing the new frame" << std::endl;
    ui.imageLabel->setPixmap(pixMap);
}

void MainWindow::updateView()
{
    if (!cap.isOpened()) {
        start_cam_capture();
    }

    // We have waited our given pause
    if (let_pic_rest && rest_counter > MS_PAUSE) {
        std::cout << "Our rest-counter got reset" << std::endl;
        rest_counter = 0;
        let_pic_rest = false;
        cv::Mat img = get_image();
        swap_image(img);
    }

    // Time has elapsed long enough that there should be a new image
    if (!let_pic_rest && rest_counter > 1000/CAM_FPS) {
        rest_counter = 0;
        cv::Mat img = get_image();
        swap_image(img);
    }

    rest_counter += TIMER_MS;
}

/*
void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}
*/
}  // namespace compare_app

