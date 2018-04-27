#include "project3/Map_manager.hpp"

using namespace cv;

Map_manager::Map_manager(){
	
  image = imread( "/home/mayavan/planning_ws/src/project3/point_map.png", cv::IMREAD_GRAYSCALE );

  if ( !image.data )
    {
        std::cout<< "No Data " <<  std::endl;
    }
  
}

// returns the color at the given coordinate
int Map_manager::get_state(int x, int y){	
	return (int)image.at<uchar>(y,x);
}

// show the read image in a window
void Map_manager::show_image(){
  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", image ); 
  waitKey(10000);
}

// Check if the particular grid has obstacle
bool Map_manager::checkObstacle(std::vector<int> grid){
  if (get_state(grid[0],grid[1])==0)
    return true;
  else
    return false;
}

// get position in meter and return in pixel coordinates in image
std::vector<int> Map_manager::computeGridPosition(std::vector<float> position){
  std::vector<int> grid;
  grid.push_back(floor(position[0]*20));
  grid.push_back(floor(position[1]*20));
  return grid;
}