/**
 * @file SleepStageAlgorithm.hpp
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

#include "boost/date_time/posix_time/posix_time.hpp"
#include <vector>
#include <json/json.h>
#include <json/value.h>
#include "Features.hpp"
#include "DataSample.hpp"

 class SleepStageAlgorithm{

	private:
 		const static int MinRRIntervalValue = 350; // in milliseconds 
		const static int MaxRRIntervalValue = 1500; // in milliseconds
		const static float RRIntervalSamplingFrequency = 8.f; // in Hz

		const static int FFTNumberOfSamples = 1024;

 	public:
 		SleepStageAlgorithm(){}
 		~SleepStageAlgorithm(){}
 		
 		/**
 		 * @brief      process a batch of samples fetched on a specified interval [iIntervalStart, iIntervalEnd]
 		 *
 		 * @param[in]  iIntervalStart    The timestamp mesured at interval start 
 		 * @param[in]  iIntervalEnd      The timestamt mesured at interval end
 		 * @param[in/out]     ioCurrentSamples  The current interval samples
 		 */
 		void processSamples(boost::posix_time::ptime iIntervalStart, 
                    		boost::posix_time::ptime iIntervalEnd, 
                    		std::vector<DataSamplePtr>& ioCurrentSamples);

 	private:
 		/**
 		 * @brief      Filter extreme R-R interval values out of [MinRRIntervalValue, MaxRRIntervalValue]
 		 *
 		 * @param      ioCurrentSamples The current interval samples
 		 */
 		void removeExtremeValues(std::vector<DataSamplePtr>& ioCurrentSamples);
		
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
		std::vector<int> resample(boost::posix_time::ptime iIntervalStart, 
                          		  boost::posix_time::ptime iIntervalEnd,
                         		  float iSamplingFrequency,
                          		  const std::vector<DataSamplePtr>& iCurrentSamples);


		/**
		 * @brief      normalize data - center values on 0
		 *
		 * @param[in]  iResampledSamples  The resampled data
		 *
		 * @return     the normalized resampled data
		 */
		std::vector<float> normalizeSignal(const std::vector<int>& iResampledSamples);


 		/**
 		 * @brief      extract features used to characterize the sleep stage
 		 *
 		 * @param[in]  iNormalizedSamples  The normalized resampled data
 		 *
 		 * @return     extracted features
 		 */
 		Features* extractFeatures(const std::vector<float>& iNormalizedSamples);

 		/**
 		 * @brief      evaluate the sleep state based on previously extracted features
 		 *
 		 * @param      iFeatures  Textracted features
 		 */
 		void classifySleepState(Features * iFeatures);
 };