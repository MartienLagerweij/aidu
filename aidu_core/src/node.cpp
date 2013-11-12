#include <aidu_core/node.h>
#include <stdio.h>

using namespace aidu::core;

Node::Node() : nh("~") {
  printf("Node::Node()\n");
}

Node::~Node() {
  printf("Node::~Node()\n");
  nh.shutdown();
}

void Node::spin() {
  printf("Node::spin()\n");
  ros::spin();
}