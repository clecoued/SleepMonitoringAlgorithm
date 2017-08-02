/**
 * @file DataSample.hpp
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
 * physiological data model
 */


#include "DateHelper.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <json/json.h>
#include <json/value.h>
#include <boost/shared_ptr.hpp>

class DataSample
{
	private:
		int mRrInterval; // in ms
		boost::posix_time::ptime mTimestamp;

	public:
		DataSample(int iRrInterval, boost::posix_time::ptime iTimestamp)
		{
			mRrInterval = iRrInterval;
			mTimestamp = iTimestamp;
		}

		DataSample(Json::Value iJsonSample)
		{
			mRrInterval = iJsonSample["rr_interval"].asInt();
			mTimestamp = DateHelper::jsonToPtime(iJsonSample);
		}

		~DataSample(){}

		int getRrInterval()
		{
			return mRrInterval;
		}

		boost::posix_time::ptime getTimestamp()
		{
			return mTimestamp;
		}
};

typedef boost::shared_ptr<DataSample> DataSamplePtr;
