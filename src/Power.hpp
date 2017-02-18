#include <iostream>
#include <memory>
#include <iostream>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <cfenv>
//#include "dyspanradio.h"
//#include "spectrum.h"
//#include "socket.h"
#include "database_comms.h"
#include <boost/circular_buffer.hpp>
#define CHANGE 0
#define WAIT 1


class PowerSearcher
{
public:
    //PowerSearcher(uhd::usrp::multi_usrp::sptr usrp_ptr, spectrum* spec_ptr, int InitialPower, float Factor = 0.5)
    //PowerSearcher(spectrum* spec_ptr, int InitialPower, float Factor = 0.5)
    PowerSearcher(int InitialPower, float Factor = 0.5)
    {
        CurrentPower = InitialPower;
        ScalingFactor = Factor;
        //usrp = usrp_ptr;
        //spec = spec_ptr;
        state = CHANGE;
        CurrentPuThru = 0;
        MaxPuThru = 0;
        count = 0;
        PreviousPuThru = 0;
        PreviousMaxPuThru = 0;


        CurrentSuThru = 0;
        MaxSuThru = 0;
    
        PreviousSuThru = 0;
        PreviousMaxSuThru = 0;
        PowerHist.set_capacity(6);
       
        for(int i = 0; i<6; i++)
            PowerHist.push_back(0);
        
        
        Score = 0;
        ScorePrev = 0;
        direction = true;
        hrys = Score*(0.05);
         
        
        
    };
    ~PowerSearcher()
    {};

    int CCompute(int Power)
    {
        count++;
        if (count == 200)
        {
            count = 0;
            return Compute(Power);
        }
        else
        {
            return Power;
        }
        
    }
    int  Compute(int Power)
    {
        PreviousPower = CurrentPower;
        CurrentPower = Power;
        
        PreviousMaxPuThru = MaxPuThru;
        PreviousPuThru = CurrentPuThru;
        
        PreviousMaxSuThru = MaxSuThru;
        PreviousSuThru = CurrentSuThru;
        int myRadio;
        //myRadio = spectrum_getRadioNumber(spec);
        //spectrum_getThroughput(spectrum object pointer,radio number, duration (ms))
        
        
        //CurrentPuThru = spectrum_getThroughput(spec, 1,  10);
        CurrentPuThru = DatabaseApi::getInstance().Tpu();
        
        
        //MaxPuThru = spectrum_getProvidedThroughput(spec,1,10);
        MaxPuThru = DatabaseApi::getInstance().Tpu_provided();
        
        
        
        //CurrentSuThru = spectrum_getThroughput(spec, 1,  10);
        CurrentSuThru = DatabaseApi::getInstance().Tsu();
        
        //MaxSuThru = spectrum_getProvidedThroughput(spec, 1, 10);
        MaxSuThru = DatabaseApi::getInstance().Tsu_provided();
        
        
        
        std::cout << "[IN POWER ] SU throughput is :  "  << CurrentSuThru << "  out of a possible : " << MaxSuThru << std::endl;
        std::cout << "[IN POWER ]  PU throughput is :  "  << CurrentPuThru << "  out of a possible : " << MaxPuThru << std::endl;
        ScorePrev = Score;
        
        double expon = -10*((MaxPuThru - CurrentPuThru)/MaxPuThru);
      
        Score = CurrentSuThru*std::exp(expon);
       
        double totScore = MaxSuThru * exp(-10*(0));
        
        double ScoreDiff = Score - ScorePrev;
        double newPower;
        hrys = Score*(0.1);
        switch(state)
        {
            case CHANGE:
                if(ScoreDiff < 0 && fabs(ScoreDiff) > hrys)
                {
                    //double adjustment = ScalingFactor*Power*(fabs(ScoreDiff)/totScore); 
                    //double adjustment = ScalingFactor*Power*(fabs(ScoreDiff)/ScorePrev);
                    
                    
                    newPower = (direction) ? Power - 1 : Power + 1; 
                    
                    PowerHist.push_back(newPower);
                    //transitions.push_back(false);
                    if(
                       (PowerHist.at(5) == PowerHist.at(3)) &&
                      (((PowerHist.at(4) - PowerHist.at(5) == 1) && direction) || ((PowerHist.at(5) - PowerHist.at(4) == 1) && !direction) ) 
                      )
                        
                    {
                        state = WAIT;
                        std::cout << "************** go to wait state*****************" << std::endl;
                    }
                    direction = !direction;
                    //return newPower;
                }
                else// wrong
                {
                    //transitions.push_back(true);
                    newPower = (direction) ? Power + 1 : Power - 1;
                    PowerHist.push_back(newPower);
                    
                }
                
                
            break; 
            case WAIT:
                if(ScoreDiff < 0 && fabs(ScoreDiff) > hrys)// might want some kind of hyresthesis to allow for noisey throughput calc
                {
                    newPower = (direction) ? Power - 1 : Power + 1;
                    PowerHist.push_back(newPower);
                    state = CHANGE;
                }
                else
                {
                    newPower = Power;
                    //find logic to decide to search again
                }
                
            break;
            
            default:
            
            break;
        }
       
        return newPower;
        
    };
    
    double function(double power)
    { 
        double x = ((0.05)*-(power-25)*(power-25))/25+1;
        return x;
    };
    private:
    spectrum* spec;
    double PreviousPower;
    double CurrentPower;
    //uhd::usrp::multi_usrp::sptr usrp;
    int count;
    double CurrentPuThru;
    double MaxPuThru;
    
    double PreviousPuThru;
    double PreviousMaxPuThru;


    double CurrentSuThru;
    double MaxSuThru;
    
    double PreviousSuThru;
    double PreviousMaxSuThru;
    
    double Score;
    double ScorePrev;
    int state;
    double ScalingFactor;
    boost::circular_buffer<double> PowerHist;
    bool direction;
    double hrys;
    
};
