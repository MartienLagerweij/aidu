#include <aidu_core/node.h>
#include <stdio.h>

using namespace aidu::core;

Node::Node() {
  nh = new ros::NodeHandle("~");
  printf("Node::Node()\n");
}

Node::~Node() {
  printf("Node::~Node()\n");
  nh->shutdown();
  delete nh;
}

void Node::spin() {
  printf("Node::spin()\n");
  ros::spin();
}
