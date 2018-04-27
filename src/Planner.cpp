#include "project3/Planner.hpp"

Planner::Planner(){
  t = 0.5;
  l = 0.287;

  pi = 3.1415926535897;

  debug = false;
  
  float arr[7][2] = {{0,0.13},{0,0.26},{0.13,0},{0.26,0.26},{0.13,0.26},{0.26,0},{0.26,0.13}};
  for(int i=0;i<7;i++){
    for(int j=0;j<2;j++){
      possible_velocity[i][j] = arr[i][j];
    }
  }
}

// get the possible actions from given states, considering obstacles and occupancy
std::vector<std::vector<float> > Planner::get_possible_new_positions(std::vector<float> state, Map_manager m){
  std::vector<std::vector<float> > output;

  for(int i=0; i<7;i++){
    std::vector<float> position = compute_resultant_position(possible_velocity[i], state);
    if (!m.checkObstacle(m.computeGridPosition(position)))
      // Appending the left and right velocity to the position for access
      position.push_back(possible_velocity[i][0]);
      position.push_back(possible_velocity[i][1]);
      
      output.push_back(position);
  }
  return output;
}

std::vector<float> Planner::compute_resultant_position(float velocity[2], std::vector<float> state){
  float x = state[0];
  float y = state[1];
  float theta = state[2];

  std::vector<float> out;

  if(velocity[0]==velocity[1])
  {
    float distance = velocity[0]*0.5;
    out.push_back(x+distance*cos(theta));
    out.push_back(y+distance*sin(theta));
    out.push_back(theta);
    return out;
  }

  float R = 0.5*l*((velocity[0]+velocity[1])/(velocity[1]-velocity[0]));
  float omega = (velocity[1]-velocity[0])/l;

  float ICCx = x - R * sin(theta);
  float ICCy = y + R * cos(theta);

  float x_new = (cos(omega * t) * (x - ICCx))-(sin(omega * t) * (y - ICCy)) + ICCx;
  float y_new = (sin(omega * t) * (x - ICCx))+(cos(omega * t) * (y - ICCy)) + ICCy;
  float theta_new = theta + omega * t;

  out.push_back(x_new);
  out.push_back(y_new);
  out.push_back(theta_new);
  return out;
}

// Calculates the cost to go for every node in the map
std::vector<std::vector<float> >  Planner::calculateHeuristics(std::vector<int> target_point){
  std::vector<std::vector<float> > heuristics;

  for (int xx=0;xx<204;xx++){
    std::vector<float> temp;
    for (int yy=0;yy<223;yy++){
        std::vector<float> point;
        point.push_back(xx);
        point.push_back(yy);
        
        std::vector<float> target;
        target.push_back(target_point[0]*0.05+0.025);
        target.push_back(target_point[1]*0.05+0.025);
        temp.push_back(calculateDistance(target, point));
    }
    heuristics.push_back(temp);
  }
  return heuristics;
}

float Planner::calculateDistance(std::vector<float> first_point, std::vector<float> second_point){
  return (float) sqrt ((double) pow(first_point[0]-second_point[0],2)+ (double) pow(first_point[1]-second_point[1],2));
}