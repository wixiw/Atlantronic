/*
 * FilteringService.cpp
 *
 *  Created on: 6 nov. 2010
 *      Author: ard
 */

#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

using namespace RTT;

class FilteringService : public Service
{
public:
    FilteringService(TaskContext* c):
        Service("filtering",c)
    {
        addOperation("firstDerivateLimitation", &FilteringService::firstDerivateLimitation, this, ClientThread )
                .doc("Saturate the input's first derivate in [-vmin;vmax]")
                .arg("input","current value of the signal you what to filter en IN_UNIT")
                .arg("lastOutput", "last value of the filtered signal en IN_UNIT")
                .arg("period" , "time between input and lastOutput, shall not be null en TIME_UNIT")
                .arg("vmin","minimal value of the first derivate (maybe be either positive or negative) en IN_UNIT/TIME_UNIT")
                .arg("vmax","maximal value of the first derivate (maybe be either positive or negative) en IN_UNIT/TIME_UNIT");
    }


    double firstDerivateLimitation(double input, double lastOutput, double period, double vmin, double vmax)
    {
        double output=0;
        double derivate=0;

        if( period > 0 && vmin < vmax)
        {
            //calcul de la derivée
            derivate = (input - lastOutput)/period;


            //filtrage
            if( derivate > vmax )
                output = lastOutput + vmax*period;
            else if( derivate < vmin )
                output = lastOutput + vmin*period;
            else
                output = input;
        }
        else
        {
            output = lastOutput;
        }

        return output;
    }
};


ORO_SERVICE_NAMED_PLUGIN(FilteringService, "filtering")
