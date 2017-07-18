/**
 * @file Features.cpp
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
 * Features class is storing features to inject to sleep stage classification algorithm
 */

#include "Features.hpp" 


/**
 * @brief      Constructs the object.
 *
 * @param[in]  iLowFreqBandPower   The low frequency band power
 * @param[in]  iHighFreqBandPower  The high frequency band power
 */
Features::Features(double iLowFreqBandPower, double iHighFreqBandPower){
	mLowFreqBandPower = iLowFreqBandPower;
	mHighFreqBandPower = iHighFreqBandPower;
}