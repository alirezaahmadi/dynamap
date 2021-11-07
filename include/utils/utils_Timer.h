/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <sys/time.h>

namespace DynaMap{
    namespace utils{

        static struct timeval startEpoch, endEpoch, nowEpoch;
        static struct timezone timeZone;

        static void timerStart(void){
            gettimeofday(&startEpoch, &timeZone);
        }

        static double timerNow(void){
            gettimeofday(&nowEpoch,&timeZone);
            return (double)nowEpoch.tv_sec + (double)nowEpoch.tv_usec/(1000*1000);
        }
        
        static void timerStop(void){
            gettimeofday(&endEpoch,&timeZone);
        }

        static double tval(){
            double _start, _end;
            _start =  (double)startEpoch.tv_sec + (double)startEpoch.tv_usec/(1000*1000);
            _end =  (double)endEpoch.tv_sec + (double)endEpoch.tv_usec/(1000*1000);
            return _end-_start;
        }

    } // end of namespace utils

}
