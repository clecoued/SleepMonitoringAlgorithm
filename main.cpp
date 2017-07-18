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
#include <boost/algorithm/string/replace.hpp>
#include <boost/foreach.hpp>
#include <fftw3.h>

const static int TimeIntervalLength = 120; // in seconds
const static int TimeIntervalShift = 20; //in seconds

const static int MinRRIntervalValue = 300; // in milliseconds 
const static int MaxRRIntervalValue = 1500; // in milliseconds
const static float RRIntervalSamplingFrequency = 8.f; // in Hz

//static boost::posix_time::ptime initialTimestamp;

boost::posix_time::ptime jsonToPtime(const Json::Value& iSample)
{
  std::string lTimestampString = iSample["Timestamp"].asString();
  boost::replace_all(lTimestampString, "T", " ");
  boost::posix_time::ptime t( boost::posix_time::time_from_string(lTimestampString) );
  return t;
}



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
    if(jsonToPtime(*it) < oIntervalStart)
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

void removeExtremeValues(std::vector<Json::Value>& ioCurrentSamples)
{
  int lCurrentRRInterval = 0;
  std::vector<Json::Value>::iterator it;

  for(it = ioCurrentSamples.begin(); it != ioCurrentSamples.end(); it++)
  {
    lCurrentRRInterval = (*it)["RrInterval"].asInt();

    if(lCurrentRRInterval < MinRRIntervalValue || lCurrentRRInterval > MaxRRIntervalValue)
    {
      //std::cout << "Remove Extreme Values " << std::endl;
      ioCurrentSamples.erase(it);
      it--;
    }
  } 
}

std::vector<float> normalizeSignal(const std::vector<int>& iResampledSamples)
{
  double lRrAverage = 0; 
  BOOST_FOREACH(int lRr, iResampledSamples)
  {
    lRrAverage += lRr;
  }

  lRrAverage = lRrAverage / iResampledSamples.size();

  std::vector<float> oNormalizedSamples; 
  oNormalizedSamples.reserve(iResampledSamples.size());

  std::vector<int>::const_iterator it = iResampledSamples.begin();
  for(;it != iResampledSamples.end(); it++)
  {
    oNormalizedSamples.push_back((*it) - lRrAverage);
  }

  BOOST_FOREACH(float lNorm, oNormalizedSamples)
  {
    std::cout << " " << lNorm << std::endl;
  }

  return oNormalizedSamples;
}

std::vector<int> resample(boost::posix_time::ptime iIntervalStart, 
                          boost::posix_time::ptime iIntervalEnd,
                          float iSamplingFrequency,
                          const std::vector<Json::Value>& iCurrentSamples)
{
  float lSamplingStep = 1.f / iSamplingFrequency * 1000; //im ms 
  std::vector<int> oResampledRRInterval;

  boost::posix_time::time_duration lTd = iIntervalEnd - iIntervalStart;
  int lNbOfSamples = lTd.total_milliseconds() * 1.0 / lSamplingStep;

  std::vector<Json::Value>::const_iterator it = iCurrentSamples.begin();

  boost::posix_time::ptime t1 = jsonToPtime(*it);
  int i1 = std::floor( (t1 - iIntervalStart).total_milliseconds() * 1.f / lSamplingStep) + 1;
  int rr1 = (*it)["RrInterval"].asInt();

  for(int i = 0; i < i1; i++)
  {
    oResampledRRInterval.push_back(rr1);
  }

  boost::posix_time::ptime t2;
  int i2 = 0, rr2 = 0;
  int rr = 0;
  boost::posix_time::ptime t;
  it++;

  for(; it != iCurrentSamples.end();it++)
  {
    t2 = jsonToPtime(*it);
    i2 = std::floor( (t2 - iIntervalStart).total_milliseconds() * 1.f / lSamplingStep);
    rr2 = (*it)["RrInterval"].asInt();

    float alpha = (rr2 - rr1) * 1.f /( (t2 - t1).total_milliseconds() );
    for(int i = (i1 + 1); i <= i2; i++)
    {
      t = iIntervalStart + boost::posix_time::milliseconds(i * lSamplingStep);
      rr = alpha * (t - t1).total_milliseconds() + rr1;
      oResampledRRInterval.push_back(rr);
    }

    t1 = t2;
    i1 = i2;
    rr1 = rr2;
  }

  for(int i = i1; i < lNbOfSamples; i++)
  {
    oResampledRRInterval.push_back(rr1);
  }

  /*boost::posix_time::ptime tFin;
  int i = 0;

  BOOST_FOREACH(int lRrValue, oResampledRRInterval)
  {
    tFin = iIntervalStart + boost::posix_time::milliseconds(i * lSamplingStep);
    i++;
    std::cout << (tFin - initialTimestamp).total_milliseconds() << " " << lRrValue << std::endl;
  }*/

  return oResampledRRInterval;
}

void extractFeatures(std::vector<float>& iNormalizedSamples)
{

}

void computeSamples(boost::posix_time::ptime iIntervalStart, 
                    boost::posix_time::ptime iIntervalEnd, 
                    std::vector<Json::Value>& ioCurrentSamples)
{
 // std::cout << "Compute Samples " <<std::endl;

 // BOOST_FOREACH(Json::Value lSample, ioCurrentSamples)
 // {
 //   std::cout << (jsonToPtime(lSample) - initialTimestamp).total_milliseconds() << " " << lSample["RrInterval"] << std::endl;
 // }


  removeExtremeValues(ioCurrentSamples);

  std::vector<int> lResampledRRIntervals = resample(iIntervalStart, 
                                                    iIntervalEnd, 
                                                    RRIntervalSamplingFrequency, 
                                                    ioCurrentSamples);

  std::vector<float> lNormalizedRRIntervals = normalizeSignal(lResampledRRIntervals);
  extractFeatures(lNormalizedRRIntervals);
}


int main ()  
{
  // parse json data from file 
  Json::Value lSleepDataJson;
  std::ifstream lSleepDataStream("data/Sommeil10MinSample.json", std::ifstream::binary);
  lSleepDataStream >> lSleepDataJson;

  // initialize interval 
  boost::posix_time::ptime lIntervalStart = jsonToPtime(lSleepDataJson[0]);
  //initialTimestamp = lIntervalStart;
  boost::posix_time::ptime lIntervalEnd = lIntervalStart + boost::posix_time::seconds(TimeIntervalLength);
  //std::cout << lIntervalStart << " " << lIntervalEnd << std::endl;

  Json::Value lCurrentSample; 
  boost::posix_time::ptime lCurrentSampleTimestamp;
  std::vector<Json::Value> lCurrentSamplesInInterval;

  for(int i = 0; i < lSleepDataJson.size(); i++)
  {
    lCurrentSample = lSleepDataJson[i];
    lCurrentSampleTimestamp = jsonToPtime(lCurrentSample);
    
    // fullfill data samples on a specific interval [lIntervalStart, lIntervalStart + TimeWindowSize]
    if(lCurrentSampleTimestamp < lIntervalEnd) 
    {
      lCurrentSamplesInInterval.push_back(lCurrentSample);
    }
    // process data and move to next interval
    else 
    {
      computeSamples(lIntervalStart, lIntervalEnd, lCurrentSamplesInInterval);

      moveInterval(lIntervalStart, lIntervalEnd, lCurrentSamplesInInterval, TimeIntervalShift); 
    }

  }

  return 0;

}