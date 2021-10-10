#ifndef ORCA_TOPSIDE_GST_UTIL_HPP
#define ORCA_TOPSIDE_GST_UTIL_HPP

extern "C" {
#include "gst/gst.h"
}

#include <vector>

namespace gst_util
{

void copy_buffer(GstBuffer *buffer, std::vector<unsigned char>& dest);

}  // namespace gst_util

#endif  // ORCA_TOPSIDE_GST_UTIL_HPP
