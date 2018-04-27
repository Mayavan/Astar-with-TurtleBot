#include "project3/Map_manager.hpp"
#include "project3/Planner.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/Twist.h>

# define DEBUG false

using namespace std;

// Structure used to store the Closed Nodes
struct Nodes
{
  vector<int> point;
  vector<float> state;
  int parent_node;
  int costToCome;
  float costToGo;
  float TotalCost;
};

// Structure used to store the OpenNodes
struct OpenNodes
{
  vector<int> point;
  vector<float> state;
  int parent_node;
  int costToCome;
  float TotalCost;
};

// function to check if the given node is an openNode and also returns the position
int is_open_node(vector<int> node, vector<OpenNodes> openNodes);
// function to check if the given node is an closed Node
bool is_closed_node(vector<int> node, vector<Nodes> nodes);
// check if the node is 5 grids close to destination
bool hasReached(vector<int> currentNode, vector<int> target_point);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "project3_node");

  ros::NodeHandle node;

  ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Initialization

  Planner planner;
  Map_manager manager;

  // Initialized position corresponds to the spawn position in gazebo environment
  vector<float> start_state, end_state;
  start_state.push_back(5.675);
  start_state.push_back(5.025);
  start_state.push_back(0);
  

  vector<int> start_point, target_point;
  // target_point = manager.computeGridPosition(end_state);
  start_point = manager.computeGridPosition(start_state);

  if(DEBUG)
    cout << "Start Point: " << start_point[0] << ", " << start_point[1] << endl;

  target_point.push_back(180);
  target_point.push_back(180);

  end_state.push_back(target_point[0] * 0.05+0.025);
  end_state.push_back(target_point[1] * 0.05+0.025);

  vector<Nodes> nodes;
  vector<OpenNodes> openNodes;

  vector<int> currentNode = start_point;
  vector<float> currentState = start_state;
  int costToComeToCurrentNode = 0;
  int parentNodeNumber = 0;

  int counter = 0;

  // Start of search algorithm
  while (!hasReached(currentNode, target_point))
  {
    // Get Possible Actions
    vector<vector<float> > states = planner.get_possible_new_positions(currentState, manager);
    vector<vector<int> > visibleNodes;

    if (DEBUG)
      cout << "VisibleNodes:" << endl;
    // get the grid position for every state
    for (int i = 0; i < states.size(); i++)
    {
      visibleNodes.push_back(manager.computeGridPosition(states[i]));
      if (DEBUG)
        cout << visibleNodes[i][0] << ", " << visibleNodes[i][1] << endl;
    }

    // Calculate Costs
    vector<float> costToCome;
    vector<float> costToGo;
    for (int j = 0; j < visibleNodes.size(); j++)
    {
      // costToGo is distance from new position to target
      costToGo.push_back(planner.calculateDistance(states[j], end_state));
      // costToCome is distance from current to new position + costToComeToCurrentNode
      costToCome.push_back(costToComeToCurrentNode + planner.calculateDistance(states[j], currentState));
    }

    // Replace with Smaller cost
    for (int i = 0; i < visibleNodes.size(); i++)
    {
      int isOpen = is_open_node(visibleNodes[i], openNodes);
      int costOfVisibleNode = costToCome[i] + costToGo[i];
      if (isOpen == 0)
      {
        // Check if already closed
        bool isClosed = is_closed_node(visibleNodes[i], nodes);

        // Add to new nodes if not closed
        if (~isClosed)
        {
          // Adding a new node
          openNodes.push_back(OpenNodes());
          int lastIndex = openNodes.size() - 1;
          openNodes[lastIndex].point = visibleNodes[i];
          openNodes[lastIndex].state = states[i];
          openNodes[lastIndex].costToCome = costToCome[i];
          openNodes[lastIndex].parent_node = counter;
          openNodes[lastIndex].TotalCost = costOfVisibleNode;
        }
      } 
      else if (openNodes[isOpen].TotalCost > costOfVisibleNode)
      {
        //if it is an Opened Node and the new cost is smaller replace 
        openNodes[isOpen].state = states[i];
        openNodes[isOpen].TotalCost = costOfVisibleNode;
        openNodes[isOpen].parent_node = counter;
        openNodes[isOpen].costToCome = costToCome[i];
      }
    }

    // Sort open nodes based on cost
    std::sort(openNodes.begin(), openNodes.end(), [](const OpenNodes &a, const OpenNodes &b) {
      return a.TotalCost < b.TotalCost;
    });

    // Add the new node to closed nodes:
    nodes.push_back(Nodes());
    int lastIndex = nodes.size() - 1;
    nodes[lastIndex].point = currentNode;
    nodes[lastIndex].state = currentState;
    nodes[lastIndex].parent_node = parentNodeNumber;
    nodes[lastIndex].costToCome = costToComeToCurrentNode;
    nodes[lastIndex].costToGo = planner.calculateDistance(currentState, end_state);
    nodes[lastIndex].TotalCost = costToComeToCurrentNode + planner.calculateDistance(currentState, end_state);

    // Take next node from open nodes
    currentNode = openNodes[0].point;
    currentState = openNodes[0].state;
    costToComeToCurrentNode = openNodes[0].costToCome;
    parentNodeNumber = openNodes[0].parent_node;

    // Remove the first openNode which is taken for next iteration 
    openNodes.erase(openNodes.begin());

    cout << "Counter: " << counter << endl;
    counter = counter + 1;

    if(DEBUG)
      for(int i=0; i < openNodes.size();i++)
      {
        cout << "Printing Node " << i << endl;
        cout << openNodes[i].point[0] << ", " << openNodes[i].point[1] << endl;
        cout << openNodes[i].state[0] << ", " << openNodes[i].state[1] << ", " << openNodes[i].state[2] << endl;
        //cout << openNodes[i].costToCome << endl;
        //cout << openNodes[i].parent_node << endl;
        //cout << openNodes[i].TotalCost << endl;     
      }
    cout << "Current Node: " << currentNode[0] << ", " << currentNode[1] << endl;
  }  // End of search

  // Add the final node to closed nodes:
  nodes.push_back(Nodes());
  int lastIndex = nodes.size() - 1;
  nodes[lastIndex].point = currentNode;
  nodes[lastIndex].state = currentState;
  nodes[lastIndex].parent_node = parentNodeNumber;
  nodes[lastIndex].costToCome = costToComeToCurrentNode;
  nodes[lastIndex].costToGo = planner.calculateDistance(currentState, end_state);
  nodes[lastIndex].TotalCost = costToComeToCurrentNode + planner.calculateDistance(currentState, end_state);

  // Find the velocities
  vector<vector<float> > velocities;
  vector<vector<int> > trace_back;
  
  // Take the velocity in reverse order
  do{
    vector<float> temp;
    vector<int> temp2;
    temp.push_back(nodes[lastIndex].state[3]);
    temp.push_back(nodes[lastIndex].state[4]);
    temp2.push_back(nodes[lastIndex].point[0]);
    temp2.push_back(nodes[lastIndex].point[1]);
    velocities.push_back(temp);
    trace_back.push_back(temp2);
    lastIndex = nodes[lastIndex].parent_node;
  } while(lastIndex != 0); 

  vector<float> temp;
  temp.push_back(nodes[lastIndex].state[3]);
  temp.push_back(nodes[lastIndex].state[4]);
  velocities.push_back(temp);

  vector<int> temp2;
  temp2.push_back(nodes[lastIndex].point[0]);
  temp2.push_back(nodes[lastIndex].point[1]);
  trace_back.push_back(temp2);



  // Writing left and right velocities to a text file
  ofstream vel, twist;
  vel.open ("velocities.txt");
  twist.open("twists.txt");
  
  for(int i=velocities.size()-1; i >= 0; i--)
  {
    vel << velocities[i][0] << ", " << velocities[i][1] << endl;
  }

  // Publish velocities and write to file
  for(int i=velocities.size()-1; i >= 0; i--)
  {
    geometry_msgs::Twist msg;
    // converting left and right velocities to linear and angular velocities
    msg.linear.x = (velocities[i][0]+velocities[i][1])/2.0;
    msg.angular.z = (velocities[i][1]-velocities[i][0])/0.287;

    ROS_INFO_STREAM("Populated velocity msg:\n" << msg);
    vel << velocities[i][0] << ", " << velocities[i][1] << endl;
    twist << "linear.x = "<< msg.linear.x <<", linear.y = 0.0, linear.z = 0.0, angular.x = 0.0, angular.y = 0.0, angular.z = "<< msg.angular.z <<endl;

    cmd_pub.publish(msg);
    ros::Duration(0.5).sleep();
  }
  
  geometry_msgs::Twist msg;

  msg.linear.x = 0;
  msg.angular.z = 0;
 
  twist << "linear.x = "<< msg.linear.x <<", linear.y = 0.0, linear.z = 0.0, angular.x = 0.0, angular.y = 0.0, angular.z = "<< msg.angular.z <<endl;

  ROS_INFO_STREAM("Populated velocity msg:\n" << msg);

  cmd_pub.publish(msg);

  vel.close();
  twist.close();
  return 0;
}

// Checks if the given node is in the open node list
int is_open_node(vector<int> node, vector<OpenNodes> openNodes)
{
  for (int i = 0; i < openNodes.size(); i++)
  {
    if (openNodes[i].point == node)
    {
      if (DEBUG)
        cout << "Node: " << node[0] << ", " << node[1] << endl << "OpenNodes: " << openNodes[i].point[0] << ", " << openNodes[i].point[1] << endl;
      return i;
      // Node Exists
    }
  }
  // return 0 if not open
  return 0;
}

// Checks if the given node is in the closed node list
bool is_closed_node(vector<int> node, vector<Nodes> nodes)
{
  for (int i = 0; i < nodes.size(); i++)
  {
    if (nodes[i].point == node)
    {
      return true;
    }
  }
  return false;
}


// Check if target reached
bool hasReached(vector<int> currentNode, vector<int> target_point)
{
  if((fabs(currentNode[0]-target_point[0]) < 5) && (fabs(currentNode[1]-target_point[1])<5))
  {
    return true;
  }
  else return false;
}