#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <iostream>
#include <QMainWindow>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QProcess>
#include <QThread>
#include "ROSThread.h"
#include "std_srvs/srv/set_bool.hpp"
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <QErrorMessage>
#include <QCloseEvent>
#include <QInputDialog>
#include <signal.h>
#include <algorithm>
#include <dirent.h>
#include <ctime>
#include <chrono>
#include <string.h>
#define R2D 180/PI
#define D2R PI/180
#define POWER_CTR_DELAY 200000
#define INTENSITY_MIN 0.0
#define INTENSITY_MAX 100.0
#define INTENSITY_COLOR_MIN 0.0
#define INTENSITY_COLOR_MAX 1.0

using namespace std;

extern QMutex mutex;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:

  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
  void RosInit(rclcpp::Node::SharedPtr &n);

private slots:
  void TryClose();
  void FilePathSet();
  void Play();
  void Save();
  void Pause();
  void PlaySpeedChange(double value);
  void LoopFlagChange(int value);
  void StopSkipFlagChange(int value);
  void AutoStartFlagChange(int value);
  void SetStamp(quint64 stamp);
  void SliderValueChange(int value);
  void SliderPressed();
  void SliderValueApply();

signals:
  void setThreadFinished(bool);
private:
  QMutex mutex;
  ROSThread *my_ros_;
  Ui::MainWindow *ui_;
  QString data_folder_path_;
  bool play_flag_;
  bool pause_flag_;
  bool save_flag_;
  bool loop_flag_;
  bool stop_skip_flag_;
  int slider_value_;

  int slider_checker_;

};

#endif // MAINWINDOW_H
