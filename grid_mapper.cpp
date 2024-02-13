#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <tf/transform_listener.h>


using namespace boost::posix_time;


class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height) :
      canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));


    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("base_pose_ground_truth", 1, \
      &GridMapper::poseCallback, this);
      
    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };
  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };



  void rayTrace(int x1, int y1, int x2, int y2) {

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    
    int sx = x1 < x2 ? 1 : -1; 
    int sy = y1 < y2 ? 1 : -1;
    
    int err = dx - dy;
    int e2;
    
    while(true) {
      // Update free/unknown cells
      if(x1 >= 0 && y1 >= 0 && x1 < canvas.rows && 
        y1 < canvas.cols) {  
        if(canvas.at<char>(x1, y1) == CELL_UNKNOWN) {
          plot(x1, y1, CELL_FREE);
        }
      }  
      
      // Check if endpoint reached
      if (x1 == x2 && y1 == y2) break;
      
      // Update error term
      e2 = 2 * err;
      if (e2 > -dy) {  
        err = err - dy; 
        x1 = x1 + sx; 
      }
      if (e2 < dx) {
        err = err + dx;
        y1 = y1 + sy;
      } 
    }
  }
  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)

    int robot_x = x + canvas.rows/2; 
    int robot_y = y + canvas.cols/2;
    
    // Mark robot cell
    plot(robot_x, robot_y, CELL_ROBOT);

    // Parse laser scan data
    int num_rays = msg->ranges.size();
    double scan_fov = msg->angle_max - msg->angle_min;
    double ray_angle_inc = scan_fov / (num_rays-1);
    
    for(int i = 0; i < num_rays; i++) {

      double ray_angle = msg->angle_min + i * ray_angle_inc;
      double ray_range = msg->ranges[i];
      
      // Calculate ray endpoint  
      double ray_x = robot_x + ray_range * cos(ray_angle);
      double ray_y = robot_y + ray_range * sin(ray_angle);

      // Ray trace along line and update free/occupied 
      rayTrace(robot_x, robot_y, ray_x, ray_y);
      
      // Check if ray ended due to obstacle
      if(i < num_rays-1 && ray_range < msg->range_max) {
        plot(ray_x, ray_y, CELL_OCCUPIED); 
      }
    }
  };
  
  
  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = -msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove following demo code and make robot move around the environment
      // plot(x, y, rand() % 255); // Demo code: plot robot's current position on canvas
      // plotImg(0, 0, CELL_OCCUPIED); // Demo code: plot different colors at 4 canvas corners
      // plotImg(0, canvas.rows-1, CELL_UNKNOWN);
      // plotImg(canvas.cols-1, 0, CELL_FREE);
      // plotImg(canvas.cols-1, canvas.rows-1, CELL_ROBOT);
      if(rand()%10 == 0) {
        double lin_vel = (rand()%100)/100.0 * FORWARD_SPEED_MPS; 
        double ang_vel = (rand()%100)/100.0 * ROTATE_SPEED_RADPS;
        move(lin_vel, ang_vel);
      }
    
      // Save map snapshot every 30 seconds 
      if(rand()%SPIN_RATE_HZ == 0) {
        saveSnapshot();
      }
      

      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  constexpr static double FORWARD_SPEED_MPS = 2.0;
  constexpr static double ROTATE_SPEED_RADPS = M_PI/2;
  
  const static int SPIN_RATE_HZ = 30;
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper abc(n, width, height); // Create new grid mapper object
  abc.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
