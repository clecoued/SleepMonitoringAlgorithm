/**
 * @file main.cpp
 * @author  clecoued <clement.lecouedic@aura.healthcare>
 * @version 1.0
 *
 *
 * @section LICENSE
 *
 * Sleep Stage Algorithm
 * Copyright (C) 2017 Aura Healthcare
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 *
 * @section DESCRIPTION
 *
 * refer to Overview.mkd to get a detailed description of the algorithm
 */

#include <iostream>
#include <fstream>
#include <json/json.h>
#include <json/value.h>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "SleepStageAlgorithm.hpp"
#include "DateHelper.hpp"

const static int TimeIntervalLength = 120; // in seconds
const static int TimeIntervalShift = 20; //in seconds

//static boost::posix_time::ptime initialTimestamp;

void moveInterval(boost::posix_time::ptime& oIntervalStart, 
                  boost::posix_time::ptime& oIntervalEnd,
                  std::vector<Json::Value>& oCurrentIntervalSamples,
                  int iTimeIntervalShift )
{
  oIntervalStart += boost::posix_time::seconds(iTimeIntervalShift);
  oIntervalEnd += boost::posix_time::seconds(iTimeIntervalShift);

  //std::cout << "Move Interval Before - " << oCurrentIntervalSamples.size() << std::endl;
  std::vector<Json::Value>::iterator it;
  for(it = oCurrentIntervalSamples.begin(); it != oCurrentIntervalSamples.end(); it++)
  {
    if(DateHelper::jsonToPtime(*it) < oIntervalStart)
    {
      oCurrentIntervalSamples.erase(it);
      it--;
    }
    else
    {
      break;
    }
  }

  //std::cout << "Move Interval After - " << oIntervalStart << " " << oIntervalEnd << " " << oCurrentIntervalSamples.size() << std::endl;
}

int main ()  
{
  // parse json data from file 
  Json::Value lSleepDataJson;
  std::ifstream lSleepDataStream("data/Sommeil10MinSample.json", std::ifstream::binary);
  lSleepDataStream >> lSleepDataJson;

  // initialize interval 
  boost::posix_time::ptime lIntervalStart = DateHelper::jsonToPtime(lSleepDataJson[0]);
  //initialTimestamp = lIntervalStart;
  boost::posix_time::ptime lIntervalEnd = lIntervalStart + boost::posix_time::seconds(TimeIntervalLength);
  //std::cout << lIntervalStart << " " << lIntervalEnd << std::endl;

  Json::Value lCurrentSample; 
  boost::posix_time::ptime lCurrentSampleTimestamp;
  std::vector<Json::Value> lCurrentSamplesInInterval;


  SleepStageAlgorithm * lSleepStageAlgorithm = new SleepStageAlgorithm();

  for(int i = 0; i < lSleepDataJson.size(); i++)
  {
    lCurrentSample = lSleepDataJson[i];
    lCurrentSampleTimestamp = DateHelper::jsonToPtime(lCurrentSample);
    
    // fullfill data samples on a specific interval [lIntervalStart, lIntervalStart + TimeWindowSize]
    if(lCurrentSampleTimestamp < lIntervalEnd) 
    {
      lCurrentSamplesInInterval.push_back(lCurrentSample);
    }
    // process data and move to next interval
    else 
    {
      lSleepStageAlgorithm->processSamples(lIntervalStart, lIntervalEnd, lCurrentSamplesInInterval);

      moveInterval(lIntervalStart, lIntervalEnd, lCurrentSamplesInInterval, TimeIntervalShift); 
    }

  }

  delete lSleepStageAlgorithm;

  return 0;

}