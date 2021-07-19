#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>

#include <sstream>
#include <fstream>

#include <cstdlib>

#include "json.hpp"
#include "PID.h"

#include <string>

#include <vector>
#include <numeric>
#include <deque>


#include "twidlle.h"
#include "forward.h"

// for convenience
using nlohmann::json;
using std::string;


int main(int argc, char *argv[]) {

   const double max_velocity = 45.0;
   PID pid_throttle;
   
   uWS::Hub h;
   PID pid_steering;
   
   twidlle twidlle_;
   context ctx_;

   bool request_reset=true;

   const double Kp = ( (argc == 5) && (std::string(argv[1])=="manual") ) ? std::stod(std::string(argv[2])) : 0.21;
   const double Ki = ( (argc == 5) && (std::string(argv[1])=="manual") ) ? std::stod(std::string(argv[3])) : 0.003;
   const double Kd = ( (argc == 5) && (std::string(argv[1])=="manual") ) ? std::stod(std::string(argv[4])) : 3.1;
   const bool capture_the_image=( (argc == 5) && (std::string(argv[1])=="manual") ) ? true : false;

   if( (argc==2) && (std::string(argv[1])=="twiddle") ){
      const double twidlle_velocity = max_velocity;
      pid_throttle.Init(0.1, 0.002, 0.0);
      set_twiddle( h, twidlle_, pid_steering, pid_throttle, twidlle_velocity, ctx_ );
   }
   else {
      std::cout<<"auto{"<<Kp<<","<<Ki<<","<<Kd<<"}"<<std::endl;
      pid_steering.Init( Kp, Ki, Kd );
      /// throttle hyperparametrs has been reused from CarND-Behavioral-Cloning
      pid_throttle.Init( 0.1, 0.002, 0.0 );
      /// set desired car speed
      set_forward(h, pid_steering, pid_throttle, max_velocity, capture_the_image );
  }
  
  h.onConnection([&h, &request_reset](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    //std::cout << "60.Connected!!!" << std::endl;
    if( request_reset ){
      // reset
      std::string msg("42[\"reset\", {}]");
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      request_reset = false;
    }
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
