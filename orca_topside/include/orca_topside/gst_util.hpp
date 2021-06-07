#include <string>

extern "C" {
#include "gst/gst.h"
}

namespace gst_util
{

void print_all_src_pad_caps(const std::string & prefix, GstElement *element);
void print_all_sink_pad_caps(const std::string & prefix, GstElement *element);

void print_caps(const std::string & prefix, GstElement *element);
void print_caps(const std::string & prefix, GstElement *element, const std::string & pad_name);
void print_caps(const std::string & prefix, GstPad *pad);

}  // namespace gst_util
