#ifndef TWIDLLE_H
#define TWIDLLE_H

class twidlle {
public:
   unsigned long samples = 100;

   unsigned long  iter=0;
   int i=0;
   
   double pp[3]={0,0,0};
   double pm[3]={0,0,0};
   
   double p[3] = {0.0, 0.0, 0.0}; // {disP(gen),disP(gen),disP(gen)};
   double dp[3]= {1.0, 1.0, 1.0};
   
   double best_error = 1e6;
   
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
         dp[i] *= 1.01;
         dump("new best+");
      } 
      else {
         if( best_error > error2 ){
            best_error = error2;
            memcpy(p,pm, sizeof(p));
            dp[i] *= 1.01;
            dump("new best-");
         } 
         else {
            dp[i] *= 0.99;
            dump("all bad..");
         }
      }
      i=(i+1)%3;
      if(i==0){
         iter++;
      }
   }
};


struct context{
   std::deque<double> accum_error[2];
   unsigned long count=0;

};

extern void set_twiddle( uWS::Hub &h, twidlle &tw, PID &pid_steering, PID &pid_throttle, const double &max_velocity, context &ctx );

#endif // TWIDLLE