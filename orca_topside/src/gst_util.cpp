// MIT License
//
// Copyright (c) 2021 Clyde McQueen
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

#include <algorithm>
#include <vector>

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

    std::copy(segment_info.data, segment_info.data + segment_info.size,
      dest.begin() + (int64_t) copied);
    copied += segment_info.size;

    gst_memory_unmap(segment, &segment_info);
    gst_memory_unref(segment);
  }
}
