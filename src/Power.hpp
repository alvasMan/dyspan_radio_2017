#include <iostream>
#include <memory>
#include <iostream>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <cfenv>
#include <random>
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
    PowerSearcher(int InitialPower, int min_power = 1, int max_power = 31, float Factor = 0.5, int PowerPeriod = 500)
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
            PowerHist.push_back(1);
        }

        Score = 0;
        ScorePrev = 0;
        direction = true;
        hrys = Score*(0.1);

        wait_check = false;
        wait_check_lower = false;
        wait_check_higher= false;
        wait_power = 0;
        wait_score = 0;
        wait_score_lower = 0;
        wait_score_higher = 0;
        wait_directions_checked = 0;

        m_PowerPeriod=PowerPeriod;
        last_gain_change = std::chrono::system_clock::now();
        MaxPower=max_power;
        MinPower=min_power;

        last_dramatic_drop = std::chrono::system_clock::now();
        last_dramatic_rise = std::chrono::system_clock::now();


    };
    ~PowerSearcher()
    {};

    int CCompute(int Power)
    {
        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_gain_change).count()) > m_PowerPeriod)
        {
            last_gain_change = std::chrono::system_clock::now();
            //return Compute(Power);
            return ComputePuSu(Power);
        }
        else
        {
            return Power;
        }
        /*
        count++;
        if (count == 100)//80)
        {
            count = 0;
            return Compute(Power);
        }
        else
        {
            return Power;
        }
        */

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
        int x = (CurrentPuThru == 0)?0:1;
        Score = CurrentSuThru*std::exp(expon)*x;
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
        if((Power >= MaxPower && direction == true))
        {
            Power = MaxPower;
            direction = false;
        }
        bool change_direction = false;
        if(Score == 0)
        {
            zero_cnt++;
        }
        if(CurrentPuThru == 0)
            direction = false;


        //std::cout << "state is : " << state << std::endl;
        //std::cout << "wait check is : " << wait_check << std::endl;
        switch(state)
        {
            case CHANGE:
                if((ScoreDiff < 0 && fabs(ScoreDiff) > hrys) || change_direction)
                {
                    //double adjustment = ScalingFactor*Power*(fabs(ScoreDiff)/totScore);
                    std::cout << "change state : change direction" << std::endl;
                    //newPower = (direction) ? Power - 1 : Power + 1;
                    newPower = get_new_power(direction, Power,1);
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
                    //newPower = (direction) ? Power + 1 : Power - 1;
                    newPower = get_new_power(direction, Power,1);
                    PowerHist.push_back(newPower);
                }


            break;
            case WAIT:
                if((ScoreDiff < 0 && fabs(ScoreDiff) > hrys) && (!wait_check))// might want some kind of hyresthesis to allow for noisey throughput calc
                {
                        //newPower = (direction) ? Power - 1 : Power + 1;
                        newPower = get_new_power(direction, Power,1);
                        PowerHist.push_back(newPower);
                        state = CHANGE;
                }
                else
                {
                    if(wait_check == true)
                    {
                        if(wait_check_lower)
                        {
                            std::cout << "wait lower checked" << std::endl;
                            wait_directions_checked++;
                            wait_score_lower = Score;
                            double temp_diff = wait_score_lower - wait_score;

                            if(temp_diff > hrys)
                            {
                                std::cout << "      {go lower}" << std::endl;
                                newPower = Power;
                                direction = false;
                                wait_check = false;
                                state = CHANGE;
                                break;
                            }
                        }
                        if(wait_check_higher)
                        {
                            std::cout << "wait higher checked" << std::endl;

                            wait_directions_checked++;
                            wait_score_higher = Score;
                            double temp_diff = wait_score_higher - wait_score;
                            if(temp_diff > hrys)
                            {
                                std::cout << "      {go higher}" << std::endl;
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
                        cout << "wait power is : " << wait_power << std::endl;
                        wait_score = Score;
                        wait_power = Power;

                        newPower = get_new_power(direction, Power,1);
                        //newPower = (direction) ? Power + 1 : Power - 1;

                        wait_check_lower = (direction)?false:true;
                        wait_check_higher = (direction)?true:false;
                        wait_check = true;
                        break;

                    }

                    if(wait_directions_checked == 2)
                    {
                        std::cout << "wait directions checked " << std::endl;
                        newPower = wait_power;
                        wait_directions_checked = 0;
                        wait_check = false;
                        wait_check_higher = false;
                        wait_check_lower = false;
                        break;

                    }
                    //newPower = (wait_check_lower)? Power + 2 : Power - 2;
                    newPower = get_new_power(wait_check_lower,Power,2);
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

    int  ComputePuSu(int Power)
    {

        initialize_throughputs(Power);

        if(!wait_check)
        ScorePrev = Score;

        double expon = -10*((MaxPuThru - CurrentPuThru)/MaxPuThru);
        int x = (CurrentPuThru == 0)?0:1;
        Score = CurrentSuThru*std::exp(expon)*x;
        double totScore = MaxSuThru * exp(-10*(0));
        double ScoreDiff = Score - ScorePrev;

        if(!wait_check)
        ScoreHist.push_back(Score);



        double newPower;
        hrys = Score*(0.08);



        std::cout << "Score is :: " << Score << std::endl;
        std::cout << "Score diff is  : " << ScoreDiff << std::endl;
        std::cout << "hyrs is : " << hrys << std::endl;
        if((Power <= MinPower && direction == false))
        {
            Power = MinPower;
            direction = true;
        }
        if((Power >= MaxPower && direction == true))
        {
            Power = MaxPower;
            direction = false;
        }
        bool change_direction = false;
        if(Score == 0)
        {
            zero_cnt++;
        }
        if(CurrentPuThru == 0)
            direction = false;

        newPower = Power;
        cout<<"Power is:   "<<Power;

        //std::cout << "state is : " << state << std::endl;
        //std::cout << "wait check is : " << wait_check << std::endl;
        if(CurrentPuThru/MaxPuThru > 0.95 || CurrentSuThru/MaxSuThru < 0.05 )
        {
            if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_dramatic_rise).count()) > 300)
            {
                
                direction=true;
                newPower = get_new_power(direction, Power,1);
                PowerHist.push_back(newPower);
                state = CHANGE;
                cout<<"DRAMA!!! Gain too low, new gain"<<newPower<<endl;
                last_dramatic_rise = std::chrono::system_clock::now();
            }
        }
        else if(CurrentPuThru/MaxPuThru < 0.8)
        {
            if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_dramatic_drop).count()) > 200)
            {
                direction=false;
                newPower = get_new_power(direction, Power,4);
                PowerHist.push_back(newPower);
                state = CHANGE;
                cout<<"DRAMA!!! Gain too high, new gain"<<newPower<<endl;
                last_dramatic_drop = std::chrono::system_clock::now();
            }
        }
        else
        {
            switch(state)
            {
            case CHANGE:
                cout << "In Change state!!" <<endl;
                if((ScoreDiff < 0 && fabs(ScoreDiff) > hrys) || change_direction)
                {
                    //double adjustment = ScalingFactor*Power*(fabs(ScoreDiff)/totScore);
                    std::cout << "change state : change direction" << std::endl;
                    //newPower = (direction) ? Power - 1 : Power + 1;
                    newPower = get_new_power(direction, Power,1);
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
                    //newPower = (direction) ? Power + 1 : Power - 1;
                    newPower = get_new_power(direction, Power,1);
                    PowerHist.push_back(newPower);
                }


            break;
            case WAIT:
                cout << "In wait state!!" <<endl;
                if((ScoreDiff < 0 && fabs(ScoreDiff) > hrys) && (!wait_check))// might want some kind of hyresthesis to allow for noisey throughput calc
                {
                        //newPower = (direction) ? Power - 1 : Power + 1;
                        newPower = get_new_power(direction, Power,1);
                        PowerHist.push_back(newPower);
                        state = CHANGE;
                }
                else
                {
                    if(wait_check == true)
                    {
                        if(wait_check_lower)
                        {
                            std::cout << "wait lower checked" << std::endl;
                            wait_directions_checked++;
                            wait_score_lower = Score;
                            double temp_diff = wait_score_lower - wait_score;

                            if(temp_diff > hrys)
                            {
                                std::cout << "      {go lower}" << std::endl;
                                newPower = Power;
                                direction = false;
                                wait_check = false;
                                state = CHANGE;
                                break;
                            }
                        }
                        if(wait_check_higher)
                        {
                            std::cout << "wait higher checked" << std::endl;

                            wait_directions_checked++;
                            wait_score_higher = Score;
                            double temp_diff = wait_score_higher - wait_score;
                            if(temp_diff > hrys)
                            {
                                std::cout << "      {go higher}" << std::endl;
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
                        cout << "wait power is : " << wait_power << std::endl;
                        wait_score = Score;
                        wait_power = Power;

                        newPower = get_new_power(direction, Power,1);
                        //newPower = (direction) ? Power + 1 : Power - 1;

                        wait_check_lower = (direction)?false:true;
                        wait_check_higher = (direction)?true:false;
                        wait_check = true;
                        break;

                    }

                    if(wait_directions_checked == 2)
                    {
                        std::cout << "wait directions checked " << std::endl;
                        newPower = wait_power;
                        wait_directions_checked = 0;
                        wait_check = false;
                        wait_check_higher = false;
                        wait_check_lower = false;
                        break;

                    }
                    //newPower = (wait_check_lower)? Power + 2 : Power - 2;
                    newPower = get_new_power(wait_check_lower,Power,2);
                    wait_check_lower = !wait_check_lower;
                    wait_check_higher = !wait_check_higher;

                    //newPower = Power;
                    //find logic to decide to search again
                }
            break;

            default:

            break;
            }
        }
       std::cout << "newPower is : " << newPower << std::endl;
        return newPower;

    };

    double function(double power)
    {
        double x = ((0.05)*-(power-25)*(power-25))/25+1;
        return x;
    };

    double get_new_power(bool direction, double power, double increment)
    {
        double new_power = (direction) ? power + increment : power - increment;
        if(new_power > MaxPower)
        {
            new_power = MaxPower;
        }
        else if (new_power < MinPower)
        {
            new_power = MinPower;
        }
        return new_power;
    };

    void initialize_throughputs(int Power)
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
        std::cout << "[IN POWER ] SU rate is :  "  << CurrentSuThru << "  out of : " << MaxSuThru << " ratio: "<< CurrentSuThru/MaxSuThru <<std::endl;
        std::cout << "[IN POWER ]  PU rate is :  "  << CurrentPuThru << "  out of : " << MaxPuThru << " ratio: "<< CurrentPuThru/MaxPuThru <<std::endl;
        return;
    }

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
    int wait_power;
    double wait_score_lower;
    double wait_score_higher;
    int wait_directions_checked;

    int m_PowerPeriod;
    std::chrono::time_point<std::chrono::system_clock> last_gain_change;
    double MaxPower;
    double MinPower;
    std::chrono::time_point<std::chrono::system_clock> last_dramatic_rise;
    std::chrono::time_point<std::chrono::system_clock> last_dramatic_drop;

};

class RampPowerChanger
{
public:
    int m_min_gain;
    int m_max_gain;
    double m_period_msec;
    
    std::chrono::system_clock::time_point last_gain_change;
    
    RampPowerChanger(int min_gain, int max_gain, double period_msec)
    {
        m_min_gain = min_gain;
        m_max_gain = max_gain;
        m_period_msec = period_msec;
        last_gain_change = std::chrono::system_clock::now();
    }
    
    int CCompute(int current_gain)
    {
        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_gain_change).count()) > m_period_msec)
        {
            last_gain_change = std::chrono::system_clock::now();
            int new_gain = current_gain+1;
            if(new_gain > m_max_gain)
                new_gain = m_min_gain;
            return new_gain;
        }
        else
        {
            return current_gain;
        }
    }
};

class RandomPowerChanger
{
public:
    int m_min_gain;
    int m_max_gain;
    double m_period_msec;
    
    std::mt19937 rng;
    std::uniform_int_distribution<std::mt19937::result_type> dist;
    std::chrono::system_clock::time_point last_gain_change;
    
    RandomPowerChanger(int min_gain, int max_gain, double period_msec) : dist(min_gain, max_gain)
    {
        m_min_gain = min_gain;
        m_max_gain = max_gain;
        m_period_msec = period_msec;
        last_gain_change = std::chrono::system_clock::now();
        
        rng.seed(std::random_device()());
    }
    
    int CCompute(int current_gain)
    {
        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_gain_change).count()) > m_period_msec)
        {
            last_gain_change = std::chrono::system_clock::now();
            return dist(rng);
        }
        else
        {
            return current_gain;
        }
    }
};