#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <ros/ros.h>
#include <aidu_card_scanner/scanner.h>
#include <aidu_user_management/Authenticate.h>

using namespace aidu;

card_scanner::Scanner::Scanner() : core::Node::Node()
{
  // Setup RFID scanner device settings
  rfidDevice = sizeof (struct input_event);
  std::string device = "/dev/input/by-id/usb-OEM_RFID_Keyboard_Emulator-event-kbd";
  
  // Open device event file
  if ((rfidDevice = open (device.c_str(), O_RDONLY)) == -1) {
    ROS_ERROR("%s is not a valid device.\n", device.c_str());
  }
  
  // Create service client
  ros::service::waitForService("authenticate");
  this->authenticate = nh.serviceClient<aidu_user_management::Authenticate>("/authenticate");
  
  // Start reading RFID input in a separate thread
  thread = new boost::thread(&card_scanner::Scanner::readRfidInput, this);
}

card_scanner::Scanner::~Scanner()
{
  close(rfidDevice);
  thread->interrupt();
  delete thread;
}

void card_scanner::Scanner::readRfidInput()
{
  
  // Define keymap and identifier string
  char keymap[] = {  0,   0, '1', '2', '3', '4', '5', '6', '7', '8',
                   '9', '0',   0,   0,   0,   0, 'Q', 'W', 'E', 'R',
                   'T', 'Y', 'U', 'I', 'O', 'P',   0,   0,   0,   0,
                   'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ';',
                     0,   0,   0,   0, 'Z', 'X', 'C', 'V', 'B', 'N',
                   'M', ',', '.', '/',   0,   0,   0, ' ',   0};
  std::string id = "";
  
  // Keep reading bytes from device
  struct input_event event;
  while(read(rfidDevice, &event, sizeof(struct input_event))) {
    
    // Only process key press events
    if(event.type == EV_KEY && event.value == 1) {
      if (event.code == 28) {
        
        // When the device presses enter, it terminates its current string, meaning an ID was read
        ROS_INFO("Logging in user with ID [%s]", id.c_str());
        
        // Send id to authenticate and reset id for the next run
        aidu_user_management::Authenticate req;
        req.request.id = id;
        req.request.login = true;
        authenticate.call(req);
        id = "";
        
      } else if (keymap[event.code] != 0) {
        
        // Add the currently typed character to the identifier as long as it isn't 0 (this ignores
        // modifiers such as shift)
        id += keymap[event.code];
        
      }
    }
  }
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "card_scanner");
  card_scanner::Scanner scanner;
  scanner.spin();
  return 0;
}