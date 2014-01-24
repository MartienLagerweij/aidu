#ifndef AIDU_ELEVATOR__ACTIONS__PUSHBUTTON_H
#define AIDU_ELEVATOR__ACTIONS__PUSHBUTTON_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_elevator/Button.h>
#include <tf/transform_listener.h>
#include <aidu_vision/DistanceSensors.h>
#include <sensor_msgs/JointState.h>

namespace aidu {
    namespace elevator {
        class PushButton : public aidu::elevator::Action {
            public:
            
                PushButton(ros::NodeHandle* nh, int button);
                ~PushButton();
                void execute();
                bool finished();
                
                void visibleButton(const aidu_elevator::Button::ConstPtr& message);
		void sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg);
		void arm_statecallback(const sensor_msgs::JointState::ConstPtr& joint_msg);
		double convert(double fov, double x, double z,double resolution);
                
            protected:
                bool buttonPushed;
                int button;
                ros::Subscriber buttonSubscriber,sensor_sub, jointState_sub;
		ros::Publisher robot_arm_positions_pub;
		tf::TransformListener listener;
		tf::StampedTransform transform;
		double arm_x,arm_y,arm_z; // position of the webcam
		double translation, rotation, extension; //joint state of the arm
		double front_left, front_right,dist_arm; //ultrasonic sensor values
		double img_x, img_y; //position values of the button (pixel)
		double horizontal_fov, vertical_fov;
		
        
        };
    }
}

#endif