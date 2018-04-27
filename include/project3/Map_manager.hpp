#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

class Map_manager {
 public:
  Map_manager();
  int get_state(int x, int y);
  void show_image();
  bool checkObstacle(std::vector<int> grid);
  std::vector<int> computeGridPosition(std::vector<float> position);

 private:
  cv::Mat image;

};
