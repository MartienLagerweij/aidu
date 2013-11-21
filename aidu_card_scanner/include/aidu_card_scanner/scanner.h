#ifndef AIDU_CARD_SCANNER__SCANNER_H
#define AIDU_CARD_SCANNER__SCANNER_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_user_management/Authenticate.h>
#include <boost/thread.hpp>

namespace aidu {
  namespace card_scanner {
    
    /**
     * The card scanner node. This node reads input from the card scanner
     * and requests authentication from the /authenticate service.
     */
    class Scanner : public aidu::core::Node {
    public:
      
      /**
       * The constructor, creating the scanner object.
       */
      Scanner();
      
      /**
       * The destructor, destroying the scanner object.
       */
      ~Scanner();
      
      /**
       * This continuous loop reads input from the database and requests
       * authentication whenever a campus card is read in.
       */
      void readRfidInput();
      
    protected:
      
      int rfidDevice;                  //!< The rfid device handle
      ros::ServiceClient authenticate; ///< The authenticate service client
      boost::thread* thread;           ///< The thread that runs the Scanner::readRfidInput()
      
    };
  }
}

#endif