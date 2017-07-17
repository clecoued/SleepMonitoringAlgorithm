// writing on a text file
#include <iostream>
#include <fstream>
#include <json/json.h>
#include <json/value.h>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/algorithm/string/replace.hpp>
#include <boost/foreach.hpp>


const static int TimeIntervalLength = 120; // in seconds
const static int TimeIntervalShift = 20; //in seconds

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

  std::cout << "Move Interval Before - " << oCurrentIntervalSamples.size() << std::endl;
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

    std::cout << "Move Interval After - " << oIntervalStart << " " << oIntervalEnd << " " << oCurrentIntervalSamples.size() << std::endl;
    std::cout << " --- DATA ---" << std::endl;
    
    BOOST_FOREACH(Json::Value lSample, oCurrentIntervalSamples)
    {
      std::cout << jsonToPtime(lSample) << " " << std::endl;
    }

    std::cout << " --- DATA ---<<" << std::endl;

}

/*void removeExtremeValues(const std::vector<Json::Value>& iCurrentSamples)
{
  std::cout << (*it)["RrInterval"].asInt() << std::endl;
}

std::vector<int> resample(const std::vector<Json::Value>& iCurrentSamples)
{

}*/

void computeSamples(boost::posix_time::ptime iIntervalStart, 
                    boost::posix_time::ptime iIntervalEnd, 
                    const std::vector<Json::Value>& iCurrentSamples)
{
  std::cout << "Compute Samples " <<std::endl;
 /* BOOST_FOREACH(Json::Value lSample, iCurrentSamples)
  {
    std::cout << lSample << " " << std::endl;
  }*/

}

int main ()  
{
  Json::Value lSleepDataJson;
  std::ifstream lSleepDataStream("data/Sommeil10MinSample.json", std::ifstream::binary);
  lSleepDataStream >> lSleepDataJson;

  std::cout << lSleepDataJson.size(); 
  boost::posix_time::ptime lIntervalStart = jsonToPtime(lSleepDataJson[0]);
  boost::posix_time::ptime lIntervalEnd = lIntervalStart + boost::posix_time::seconds(TimeIntervalLength);
  std::cout << lIntervalStart << " " << lIntervalEnd << std::endl;

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
      //std::cout << " - " << lCurrentSampleTimestamp << std::endl;
      //lSleepDataInInterval.push_back(lSleepDataJson[i])
    }
    else 
    {
      //std::cout << " + " << lCurrentSampleTimestamp << std::endl;
      computeSamples(lIntervalStart, lIntervalEnd, lCurrentSamplesInInterval);

      moveInterval(lIntervalStart, lIntervalEnd, lCurrentSamplesInInterval, TimeIntervalShift); 

      //cleanDataOutOfInterval();

    }

  }

  return 0;

}