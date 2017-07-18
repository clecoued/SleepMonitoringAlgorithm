/**
 * @file SleepStageAlgorithm.cpp
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

#include "SleepStageAlgorithm.hpp"
#include <boost/foreach.hpp>
#include <fftw3.h>
#include "DateHelper.hpp"

/**
 * @brief      process a batch of samples fetched on a specified interval [iIntervalStart, iIntervalEnd]
 *
 * @param[in]  iIntervalStart    The timestamp mesured at interval start 
 * @param[in]  iIntervalEnd      The timestamt mesured at interval end
 * @param[in/out]     ioCurrentSamples  The current interval samples
 */
void SleepStageAlgorithm::processSamples(boost::posix_time::ptime iIntervalStart, 
                    boost::posix_time::ptime iIntervalEnd, 
                    std::vector<Json::Value>& ioCurrentSamples)
{
  removeExtremeValues(ioCurrentSamples);

  std::vector<int> lResampledRRIntervals = resample(iIntervalStart, 
                                                    iIntervalEnd, 
                                                    RRIntervalSamplingFrequency, 
                                                    ioCurrentSamples);

  std::vector<float> lNormalizedRRIntervals = normalizeSignal(lResampledRRIntervals);
  Features * lFeatures = extractFeatures(lNormalizedRRIntervals);
  classifySleepState(lFeatures);

  delete lFeatures;
}

/**
 * @brief      Filter extreme R-R interval values out of [MinRRIntervalValue, MaxRRIntervalValue]
 *
 * @param      ioCurrentSamples The current interval samples
 */
void SleepStageAlgorithm::removeExtremeValues(std::vector<Json::Value>& ioCurrentSamples)
{
  int lCurrentRRInterval = 0;
  std::vector<Json::Value>::iterator it;

  for(it = ioCurrentSamples.begin(); it != ioCurrentSamples.end(); it++)
  {
    lCurrentRRInterval = (*it)["RrInterval"].asInt();

    if(lCurrentRRInterval < MinRRIntervalValue || lCurrentRRInterval > MaxRRIntervalValue)
    {
      ioCurrentSamples.erase(it);
      it--;
    }
  } 
}

/**
 * @brief      resampling the current interval samples to RRIntervalSamplingFrequency
 *
 * @param[in]  iIntervalStart      The timestamp mesured at interval start
 * @param[in]  iIntervalEnd        The timestamt mesured at interval end
 * @param[in]  iSamplingFrequency  The re-sampling frequency
 * @param[in]  iCurrentSamples     The current interval samples
 *
 * @return     the resampled data
 */
std::vector<int> SleepStageAlgorithm::resample(boost::posix_time::ptime iIntervalStart, 
                                               boost::posix_time::ptime iIntervalEnd,
                                               float iSamplingFrequency,
                                               const std::vector<Json::Value>& iCurrentSamples)
{
  float lSamplingStep = 1.f / iSamplingFrequency * 1000; //im ms 
  std::vector<int> oResampledRRInterval;

  boost::posix_time::time_duration lTd = iIntervalEnd - iIntervalStart;
  int lNbOfSamples = lTd.total_milliseconds() * 1.0 / lSamplingStep;

  std::vector<Json::Value>::const_iterator it = iCurrentSamples.begin();

  boost::posix_time::ptime t1 = DateHelper::jsonToPtime(*it);
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
    t2 = DateHelper::jsonToPtime(*it);
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

  return oResampledRRInterval;
}

/**
 * @brief      normalize data - center values on 0
 *
 * @param[in]  iResampledSamples  The resampled data
 *
 * @return     the normalized resampled data
 */
std::vector<float> SleepStageAlgorithm::normalizeSignal(const std::vector<int>& iResampledSamples)
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

  return oNormalizedSamples;
}

/**
 * @brief      evaluate the sleep state based on previously extracted features
 *
 * @param      iFeatures  Textracted features
 */
Features* SleepStageAlgorithm::extractFeatures(const std::vector<float>& iNormalizedSamples)
{
  // compute FFT
  fftw_complex lSignal[FFTNumberOfSamples];
  fftw_complex lResult[FFTNumberOfSamples];

  fftw_plan lPlan = fftw_plan_dft_1d(FFTNumberOfSamples,
                                    lSignal,
                                    lResult,
                                    FFTW_FORWARD,
                                    FFTW_ESTIMATE);

  // prepare input data
  for(int i = 0; i < iNormalizedSamples.size(); i++)
  {
    // fill Real part
    lSignal[i][0] = iNormalizedSamples.at(i);
    // fill Imaginary part
    lSignal[i][1] = 0;
  }

  for(int i = iNormalizedSamples.size(); i < FFTNumberOfSamples; i++)
  {
    // fill Real part
    lSignal[i][0] = 0;
    // fill Imaginary part
    lSignal[i][1] = 0;
  }

  fftw_execute(lPlan);


  // compute Low Freq Band Power and High Freq Band Power features
  int lLowFreqMin = std::floor(Features::LowFrequencyBandMin * FFTNumberOfSamples / RRIntervalSamplingFrequency);
  int lLowFreqMax = std::floor(Features::LowFrequencyBandMax * FFTNumberOfSamples / RRIntervalSamplingFrequency);
  int lHighFreqMin = std::floor(Features::HighFrequencyBandMin * FFTNumberOfSamples / RRIntervalSamplingFrequency);
  int lHighFreqMax = std::floor(Features::HighFrequencyBandMax * FFTNumberOfSamples / RRIntervalSamplingFrequency);

  double lLowBandPower = 0.0;
  double lHighBandPower = 0.0;
  double lAmplitude = 0;
  for(int i = lLowFreqMin + 1; i <= lLowFreqMax; i++  )
  {
      lAmplitude = std::sqrt(lResult[i][0]*lResult[i][0] + lResult[i][1]*lResult[i][1] );
      lLowBandPower += lAmplitude;
  }

  for(int i = lHighFreqMin + 1; i <= lHighFreqMax; i++)
  {
    lAmplitude = std::sqrt(lResult[i][0]*lResult[i][0] + lResult[i][1]*lResult[i][1] );
    lHighBandPower += lAmplitude;
  }

  double lTotalPower = lLowBandPower + lHighBandPower;
  lLowBandPower = lLowBandPower / lTotalPower * 100; // in %
  lHighBandPower = lHighBandPower / lTotalPower * 100; // in %
  //std::cout << lLowBandPower << " " << lHighBandPower << " " << lLowBandPower / lHighBandPower << std::endl;

  fftw_destroy_plan(lPlan);

  return new Features(lLowBandPower, lHighBandPower);
}

/**
 * @brief      evaluate the sleep state based on previously extracted features
 *
 * @param      iFeatures  Textracted features
 */
void SleepStageAlgorithm::classifySleepState(Features * iFeatures)
{

}