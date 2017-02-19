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

zero_cnt = 0;
        CurrentSuThru = 0;
        MaxSuThru = 0;
    
        PreviousSuThru = 0;
        PreviousMaxSuThru = 0;
        ScoreHist.set_capacity(6);
       PowerHist.set_capacity(6);
        for(int i = 0; i<6; i++)
        {
            ScoreHist.push_back(-1);
            PowerHist.push_back(0);
        }
        
        Score = 0;
        ScorePrev = 0;
        direction = true;
        hrys = Score*(0.1);
        
        wait_check = false;
        wait_check_lower = false;
        wait_check_higher= false;
    
        wait_score = 0;
        wait_score_lower = 0;
        wait_score_higher = 0;
         
        
        
    };
    ~PowerSearcher()
    {};

    int CCompute(int Power)
    {
        count++;
        if (count == 80)
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
        
        CurrentPuThru = DatabaseApi::getInstance().Tpu();
        MaxPuThru = DatabaseApi::getInstance().Tpu_provided();
        CurrentSuThru = DatabaseApi::getInstance().Tsu();
        MaxSuThru = DatabaseApi::getInstance().Tsu_provided();      
        std::cout << "[IN POWER ] SU throughput is :  "  << CurrentSuThru << "  out of a possible : " << MaxSuThru << std::endl;
        std::cout << "[IN POWER ]  PU throughput is :  "  << CurrentPuThru << "  out of a possible : " << MaxPuThru << std::endl;
        
        
        
        if(!wait_check)
        ScorePrev = Score;
        
        double expon = -10*((MaxPuThru - CurrentPuThru)/MaxPuThru);
        Score = CurrentSuThru*std::exp(expon)*(CurrentPuThru);
        double totScore = MaxSuThru * exp(-10*(0));
        double ScoreDiff = Score - ScorePrev;
        
        if(!wait_check)
        ScoreHist.push_back(Score);
        
        
        
        double newPower;
        hrys = Score*(0.08);
        
        
        
        std::cout << "Score is :: " << Score << std::endl;
        std::cout << "Score diff is  : " << ScoreDiff << std::endl;
        std::cout << "hyrs is : " << hrys << std::endl;
        if((Power <= 0 && direction == false))
        {
            Power = 0;
            direction = true;
        }
        if((Power >= 32 && direction == true))
        {
            Power = 32;
            direction = false;
        }
        bool change_direction = false;
        if(Score == 0)
        {
            zero_cnt++;
        }
        if(CurrentPuThru == 0)
            direction = false;
        
        switch(state)
        {
            case CHANGE:
                if((ScoreDiff < 0 && fabs(ScoreDiff) > hrys) || change_direction)
                {
                    //double adjustment = ScalingFactor*Power*(fabs(ScoreDiff)/totScore); 
                    std::cout << "change state : change direction" << std::endl;
                    newPower = (direction) ? Power - 1 : Power + 1; 
                    PowerHist.push_back(newPower);
                    if(//(PowerHist.at(5) == PowerHist.at(3)) && (PowerHist.at(4) == PowerHist.at(2)) && (PowerHist.at(5) != PowerHist.at(4)) 
                       (PowerHist.at(5) == PowerHist.at(3)) &&(((PowerHist.at(4) - PowerHist.at(5) == 1) && direction) || ((PowerHist.at(5) - PowerHist.at(4) == 1) && !direction) ) 
                      )      
                    {
                        state = WAIT;
                        std::cout << "************** go to wait state*****************" << std::endl;
                    }
                    direction = !direction;
                }
                else// wrong
                {
                    std::cout << "change state : keep going" << std::endl;
                    newPower = (direction) ? Power + 1 : Power - 1;
                    PowerHist.push_back(newPower);   
                }
                
                
            break; 
            case WAIT:
                if(ScoreDiff < 0 && fabs(ScoreDiff) > hrys)// might want some kind of hyresthesis to allow for noisey throughput calc
                {
                    newPower = (direction) ? Power - 1 : Power + 1;
                    PowerHist.push_back(newPower);
                    if(!wait_check)
                        state = CHANGE;
                }
                else
                {
                    if(wait_check == true)
                    {
                        if(wait_check_lower)
                        {
                            wait_directions_checked++;
                            wait_score_lower = Score;
                            double temp_diff = wait_score_lower - wait_score;
                            if(temp_diff > hrys)
                            {
                                newPower = Power;
                                direction = false;
                                wait_check = false;
                                state = CHANGE;
                                break;
                            }
                        }
                        if(wait_check_higher)
                        {
                            wait_directions_checked++;
                            wait_score_higher = Score;
                            double temp_diff = wait_score_higher - wait_score;
                            if(temp_diff > hrys)
                            {
                                newPower = Power;
                                direction = true;
                                wait_check = false;
                                state = CHANGE;
                                break;      
                            }                         
                        }
                    }
                    else
                    {
                        wait_score = Score;
                        newPower = (direction) ? Power + 1 : Power - 1;
                        wait_check_lower = (direction)?false:true;
                        wait_check_higher = (direction)?true:false;
                        wait_check = true;
                        break;
                        
                    }
                    
                    if(wait_directions_checked == 2)
                    {
                        newPower = wait_score;
                        wait_check = false;
                        wait_check_higher = false;
                        wait_check_lower = false;
                        break;
                        
                    }
                    newPower = (wait_check_lower)? Power + 2 : Power - 2;
                    wait_check_lower = !wait_check_lower;
                    wait_check_higher = !wait_check_higher;
                    
                    //newPower = Power;
                    //find logic to decide to search again
                }
            break;
            
            default:
            
            break;
        }
       std::cout << "newPower is : " << newPower << std::endl;
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
    boost::circular_buffer<double> ScoreHist;
    bool direction;
    double hrys;
    int zero_cnt;
    
    
    
    //////wait logic
    bool wait_check;
    bool wait_check_lower;
    bool wait_check_higher;
    
    double wait_score;
    double wait_score_lower;
    double wait_score_higher;
    int wait_directions_checked;
    
    
};
