#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cstdlib>

#include "json.hpp"
#include "PID.h"
#include "aux.h"

void do_capture_image(const std::string &image){
   static unsigned long count = 0;
   std::ostringstream ss;
   ss<<"base64 -d ../data/image.txt > ../data/"<<count<<".jpg";
   std::ofstream myfile;
   myfile.open ("../data/image.txt");
   myfile<<image;
   myfile.flush();
   myfile.close();
   std::system(ss.str().c_str());
   ++count;
}


void set_forward( uWS::Hub &h, PID &pid_steering, PID &pid_throttle, const double &max_velocity, const bool &capture_image ) {
    
   // final
   // pid_steering.Init( 0.2/2, 0.004/8, 3.0/2 );

  h.onMessage([&pid_steering, 
               &pid_throttle, 
               &max_velocity,
               &capture_image]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = nlohmann::json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          const double cte = std::stod(j[1]["cte"].get<std::string>());
          const double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          //std::cout<<"cte="<<cte<<";speed="<<speed<<";angle="<<angle<<std::endl;
          
          // ---------------------------------------
          if( capture_image ){
          // capture image for writeup
            std::string image=j[1]["image"];        
            do_capture_image(image);
          }
          // ---------------------------------------
         
          /// update an steering error
          pid_steering.UpdateError(cte);
          const double total_error = pid_steering.TotalError();

          /// get steering TotalError and select:
          /// tmp=min( 1, TotalError );
          /// then max( -1.0, tmp);
          /// to keep steer value in range [-1, 1]
          const double steer_value = std::max( -1.0, std::min( 1.0, total_error ) ); 
  
          /// update throttle error
          pid_throttle.UpdateError( speed-max_velocity );
          /// get throttle TotalError
          const double throttle = pid_throttle.TotalError();
          // DEBUG1
          // std::cout << "CTE: " << cte << " Steering: " << steer_value <<":" << speed <<":throttle:"<<throttle<< std::endl;

          nlohmann::json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage
}