#include "orca_topside/gst_util.hpp"

// Copy all memory segments in a GstBuffer into dest
void gst_util::copy_buffer(GstBuffer *buffer, std::vector<unsigned char>& dest)
{
  auto num_segments = static_cast<int>(gst_buffer_n_memory(buffer));
  gsize copied = 0;
  for (int i = 0; i < num_segments; ++i) {
    GstMemory *segment = gst_buffer_get_memory(buffer, i);
    GstMapInfo segment_info;
    gst_memory_map(segment, &segment_info, GST_MAP_READ);

    std::copy(segment_info.data, segment_info.data + segment_info.size, dest.begin() + (long) copied);
    copied += segment_info.size;

    gst_memory_unmap(segment, &segment_info);
    gst_memory_unref(segment);
  }
}
