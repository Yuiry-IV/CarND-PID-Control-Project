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

#include <random>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void capture_image(const std::string image, const unsigned long count){
   std::ostringstream ss;
   ss<<"base64 -d ./image.txt > ../data/"<<count<<".jpg";
   
   std::ofstream myfile;
   myfile.open ("image.txt");
   myfile<<image;
   myfile.flush();
   myfile.close();
   std::system(ss.str().c_str());
}

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> disP(0.81, 1.19);
std::uniform_real_distribution<> disDEL(0.2, 0.8);

class twidlle {
public:
   unsigned long samples = 40;

   unsigned long  iter=0;
   int i=0;
   
   double pp[3]={0,0,0};
   double pm[3]={0,0,0};
   
   double p[3] = //{0.0, 0.0, 0.0};    
                  {disP(gen),disP(gen),disP(gen)};
   double dp[3]= {1.0, 1.0, 1.0};            // {disDEL(gen),disDEL(gen),disDEL(gen)};
   
   double best_error = 1000;
   
   std::deque<double> speeds;
   std::deque<double> angels; 
   
   void next_step(){
      memcpy(pp,p, sizeof(p));
      memcpy(pm,p, sizeof(p));
      
      pp[i]+=dp[i];      
      pm[i]-=dp[i];
   }
   
   void dump(const std::string prefix){
      std::cout<<iter<<":"<<prefix<<":"<<best_error
                <<":{"<<p[0]<<","<<p[1]<<","<<p[2]<<"}";
      std::cout<<"{"<<dp[0]<<","<<dp[1]<<","<<dp[2]<<"}\n";
   }
   
   void update_error( double error1, double error2 ) {
      if( best_error > error1 && error1 < error2 ){
         best_error = error1;
         memcpy(p,pp, sizeof(p));
         dp[i] *= 1.1;
         dump("new best+");
      } 
      else {
         if( best_error > error2 ){
            best_error = error2;
            memcpy(p,pm, sizeof(p));
            dp[i] *= 1.1;
            dump("new best-");
         } 
         else {
            dp[i] *= 0.9;
            //dump("all bad..");
         }
      }
      i=(i+1)%3;
      if(i==0){
         iter++;        
      }
   }
   
   
};

// struct telemetry {
   
   // const unsigned long SIZE=200;
   
   // std::deque<double> cte;
   // std::deque<double> speed;
   // std::deque<double> angle;
   
   // void update(double cte, double speed, double angle){
      // cte.push_back(cte);
      // speed.push_back(speed);
      // angle.push_back(angle);
      
      // if(cte.size()>SIZE) cte.pop_front();
      // if(speed.size()>SIZE) speed.pop_front();
      // if(angle.size()>SIZE) angle.pop_front();
      
   // }
   
   // std::string to_string(){
      
   // }
// };


int main(int argc, char *argv[]) {

  /*
  if( argc != 4 ){
     std::cout<<" usage "<<argv[0]<<" Kp Ki Kd\n";
     return -1;
  }
  
  const double Kp = std::stod(std::string(argv[1]));
  const double Ki = std::stod(std::string(argv[2]));
  const double Kd = std::stod(std::string(argv[3]));
  */
  
  
  std::deque<double> accum_error[2];
  
  unsigned long count=0;
  
  uWS::Hub h;


  twidlle tw;
  tw.next_step();

  PID pid_steering;
  // pid_steering.Init( Kp, Ki, Kd );

  /**
   * lesson 12 step 11->.
   */
  // pid_steering.Init( 0.2, 0.004, 3.0 );
    
  // final 
  // pid_steering.Init( 0.2/2, 0.004/8, 3.0/2 );
  
  // video_set1.gif
  // pid_steering.Init( 0.5, 0.1, 3.0 );
  
  // 03_00_01_00_00
  // pid_steering.Init( 0.1, 0.0, 0.0 );
  
  // 03_01_01_0001_00
  // pid_steering.Init( 0.1, 0.001, 0.0 ); 
  
  PID pid_throttle;
  /// throttle hyperparametrs has been reused from CarND-Behavioral-Cloning
  pid_throttle.Init( 0.1, 0.002, 0.0 );
  
  /// set desired car speed
  double max_velocity = 7.0;

  bool request_reset=true;

  h.onMessage([&pid_steering, &pid_throttle, &max_velocity, &count,
                  &accum_error,
                  &tw ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          //std::cout<<"cte="<<cte<<";speed="<<speed<<";angle="<<angle<<std::endl;
          
          /*
          // ---------------------------------------
          // capture image for writeup
          std::string image=j[1]["image"];          
          // ---------------------------------------
          */
                    
          if( accum_error[0].size() == 0 && accum_error[1].size() == 0 ){
            // reset
            //std::string msg("42[\"reset\", {}]");
            //ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            pid_steering.Init( tw.pp[0], tw.pp[1], tw.pp[2] );
            count = 0;
          }
          else if( accum_error[0].size() == tw.samples && accum_error[1].size() == 0){
            // reset
            //std::string msg("42[\"reset\", {}]");
            //ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            pid_steering.Init( tw.pm[0], tw.pm[1], tw.pm[2] );
            count = 0;
          }
          
          
          /// update an steering error
          pid_steering.UpdateError(cte);
          
          const double total_error = pid_steering.TotalError();
          
          
          if( accum_error[0].size() < tw.samples ){
             accum_error[0].push_back( cte*cte );
          }
          else if( accum_error[0].size() == tw.samples && accum_error[1].size() < tw.samples ){              
            accum_error[1].push_back( cte*cte );
          }
          
          if( accum_error[0].size()==tw.samples && accum_error[1].size()==tw.samples ){
            
            while( accum_error[0].size()>tw.samples-2 ) accum_error[0].pop_front();
            while( accum_error[1].size()>tw.samples-2 ) accum_error[1].pop_front();
            
            double rp = std::accumulate(accum_error[0].begin(),accum_error[0].end(),0.0) / accum_error[0].size();
            double rm = std::accumulate(accum_error[1].begin(),accum_error[1].end(),0.0) / accum_error[1].size();
            
            //std::cout<<accum_error[0].size()<<","<<accum_error[1].size()<<","<<rp<<","<<rm<<std::endl;
            
            tw.update_error( rp, rm ); 
            tw.next_step();
            
            tw.speeds.clear();
            tw.angels.clear();
            
            accum_error[0].clear();
            accum_error[1].clear();
            
          }
          

          /// get steering TotalError and select:
          /// tmp=min( 1, TotalError );
          /// then max( -1.0, tmp);
          /// to keep steer value in range [-1, 1]
          double steer_value = std::max( -1.0, std::min( 1.0, total_error ) ); 

  
          /// update throttle error
          pid_throttle.UpdateError( speed - max_velocity  );
          
          /// get throttle TotalError
          double throttle = pid_throttle.TotalError(); 
          
          // DEBUG1
          // std::cout << "CTE: " << cte << " Steering: " << steer_value<< std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
         // std::cout <<accum_error[0]/count<<":"<<accum_error[1]/count << std::endl;
         // std::cout <<count<<":"<< msg << std::endl;
          
          count++;          
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h, &request_reset](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
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
