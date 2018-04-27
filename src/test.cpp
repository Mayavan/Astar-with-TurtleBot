#include <iostream>
#include <string>
#include <vector>
#include "project3/Map_manager.hpp"

using namespace std;

int main() {

  vector<float> start_state;
  start_state.push_back(1);
  start_state.push_back(1);
  start_state.push_back(0);

  vector<float> end_state;
  end_state.push_back(1);
  end_state.push_back(1);
  end_state.push_back(0);

  if (start_state == end_state)
  cout<<"Equal"<<endl;

  return 0;
}