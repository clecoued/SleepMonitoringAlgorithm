/**
 * @file DateHelper.hpp
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
 * DateHelper is an helper class used to ease conversion from string iso8601 to ptime object and vice versa
 */

#include <json/json.h>
#include <json/value.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <string>
#include <boost/algorithm/string/replace.hpp>

class DateHelper{
	public:

		/**
		 * @brief      method used to extract a timestamp from a Json data sample
		 *
		 * @param[in]  iSample  data sample
		 *
		 * @return     timestamp in a boost::ptime format
		 */
		static boost::posix_time::ptime jsonToPtime(const Json::Value& iSample)
		{
		  std::string lTimestampString = iSample["time"].asString();
		  boost::replace_all(lTimestampString, "T", " ");
		  boost::replace_all(lTimestampString, "Z", "");
		  boost::posix_time::ptime t( boost::posix_time::time_from_string(lTimestampString) );
		  return t;
		}
};

