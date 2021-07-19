#include <deque>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <sstream>
#include "json.hpp"
#include "PID.h"

#include "twidlle.h"

#include "p8_func_aux.h"

/*
#include <random>
std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> disP(0.81, 1.19);
std::uniform_real_distribution<> disDEL(0.2, 0.8);
*/

void set_twiddle( uWS::Hub &h, twidlle &tw, PID &pid_steering, PID &pid_throttle, const double &max_velocity, context &ctx ){
   tw.next_step();
   
   h.onMessage([  &tw,
                  &pid_steering,
                  &pid_throttle,
                  max_velocity,
                  &ctx ]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
   {
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
             
             if( ctx.accum_error[0].size() == 0 && ctx.accum_error[1].size() == 0 ){
               // reset
               std::string msg("42[\"reset\", {}]");
               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
               pid_steering.Init( tw.pp[0], tw.pp[1], tw.pp[2] );
               ctx.count = 0;
             }
             else if( ctx.accum_error[0].size() == tw.samples && ctx.accum_error[1].size() == 0){
               // reset
               std::string msg("42[\"reset\", {}]");
               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
               pid_steering.Init( tw.pm[0], tw.pm[1], tw.pm[2] );
               ctx.count = 0;
             }
             
             if( ctx.accum_error[0].size() < tw.samples ){
                ctx.accum_error[0].push_back( cte*cte );
             }
             else if( ctx.accum_error[0].size() == tw.samples && ctx.accum_error[1].size() < tw.samples ){              
               ctx.accum_error[1].push_back( cte*cte );
             }
             
             //std::cout<<ctx.count<<','<<ctx.accum_error[0].size()<<","<<ctx.accum_error[1].size()<<std::endl;
             
             if( ctx.accum_error[0].size()>=tw.samples && ctx.accum_error[1].size()>=tw.samples ){
               
               while( ctx.accum_error[0].size()>tw.samples-2 ) ctx.accum_error[0].pop_front();
               while( ctx.accum_error[1].size()>tw.samples-2 ) ctx.accum_error[1].pop_front();
               
               double rp = std::accumulate(ctx.accum_error[0].begin(),ctx.accum_error[0].end(),0.0) / ctx.accum_error[0].size();
               double rm = std::accumulate(ctx.accum_error[1].begin(),ctx.accum_error[1].end(),0.0) / ctx.accum_error[1].size();
               
               tw.update_error( rp, rm ); 
               tw.next_step();
               
               ctx.accum_error[0].clear();
               ctx.accum_error[1].clear();
               
             }
             
             /// update an steering error
             pid_steering.UpdateError(cte);
             const double total_error = pid_steering.TotalError();
             const double steer_value = std::max( -1.0, std::min( 1.0, total_error ) );
             
             pid_throttle.UpdateError( speed - convert_max_velocity( max_velocity,cte )  );
             const double throttle = pid_throttle.TotalError();
             
             #if DEBUG1
             std::cout << "CTE:" << cte;
             std::cout << ";Steering:" << steer_value;
             std::cout <<";speed:" << speed;
             std::cout <<";throttle:"<<throttle;
             std::cout << std::endl;
             #endif 

             
             nlohmann::json msgJson;
             msgJson["steering_angle"] = steer_value;
             msgJson["throttle"] = throttle;
             auto msg = "42[\"steer\"," + msgJson.dump() + "]";
             
             ctx.count++;
             ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
           }  // end "telemetry" if
         } else {
           std::cout<<length <<std::endl;
           // Manual driving
           std::string msg = "42[\"manual\",{}]";
           ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
         }
      }// end websocket message if
   }); // end h.onMessage
}