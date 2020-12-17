// MIT License
//
// Copyright (c) 2020 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ORCA_VISION__PARAMETERS_HPP_
#define ORCA_VISION__PARAMETERS_HPP_

#include <string>

#include "ros2_shared/context_macros.hpp"

namespace orca_vision
{

#define STEREO_PARAMS \
  CXT_MACRO_MEMBER(publish_stats, bool, true) \
  CXT_MACRO_MEMBER(debug_windows, bool, false) \
  \
  CXT_MACRO_MEMBER(subscribe_best_effort, bool, true) \
  CXT_MACRO_MEMBER(publish_tf, bool, true) \
  \
  CXT_MACRO_MEMBER(odom_frame_id, std::string, "odom") \
  CXT_MACRO_MEMBER(base_frame_id, std::string, "base_link") \
  CXT_MACRO_MEMBER(lcam_frame_id, std::string, "camera_link") \
  \
  CXT_MACRO_MEMBER(detect_num_features, int, 200) /* Features to detect */ \
  CXT_MACRO_MEMBER(detect_min_features, int, 20) /* Min features to pass each stage */ \
  \
  CXT_MACRO_MEMBER(match_max_distance, double, 25.0) /* Maximum distance in matcher space */ \
  CXT_MACRO_MEMBER(match_min_disparity, double, 10.0) /* Min disparity, pixels */ \
  CXT_MACRO_MEMBER(match_max_epipolar_error, double, 10.0) /* Maximum epipolar error, pixels */ \
/* End of list */

struct Parameters
{
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
  CXT_MACRO_DEFINE_MEMBERS(STEREO_PARAMS)
};

}  // namespace orca_vision

#endif  // ORCA_VISION__PARAMETERS_HPP_
