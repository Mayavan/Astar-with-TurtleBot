#include <math.h>
#include <iostream>
#include <vector>

#include "project3/Map_manager.hpp"

class Planner {
 public:
  Planner();
  std::vector<std::vector<float> > get_possible_new_positions(std::vector<float> state, Map_manager m);
  std::vector<float> compute_resultant_position(float velocity[2], std::vector<float> state);
  std::vector<std::vector<float> >  calculateHeuristics(std::vector<int> target_point);
  float calculateDistance(std::vector<float> first_point, std::vector<float> second_point);
  double pi;

  bool debug;

 private:
  // Distance between the centre of the wheels
  float l;
  // Sampling Time in seconds
  float t;


  // pi = 3.1415926535897 and Diameter of wheel = 0.076
  // All the combination of the allowed velocity
  float possible_velocity[7][2];
};
