#ifndef AIDU_CORE__NODE_H
#define AIDU_CORE__NODE_H

#include <ros/ros.h>

namespace aidu {
  namespace core {
    /**
     * The node is an abstract base class for ROS nodes. It sets up a node handle
     * and offers  default spin() method, which can be overridden. Overriding the
     * constructor makes it possible to customize the initialization of the class.
     */
    class Node {
      
    protected:
      
      /// The node handle
      ros::NodeHandle nh;
      
    public:
      
      /// Creates a new node
      Node();
      
      /// Destroys an existing node
      virtual ~Node();
      
      /**
       * This method should contain the ROS logic.
       * The default implementation places ROS in a message handling
       * loop (ros::spin()).
       */
      virtual void spin();
      
    };
  }
}

#endif