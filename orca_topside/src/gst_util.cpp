#include "orca_topside/gst_util.hpp"

#include <iostream>

namespace gst_util
{

void print_all_src_pad_caps(const std::string & prefix, GstElement *element)
{
  if (!element) {
    return;
  }

  GstIterator *iter = gst_element_iterate_src_pads(element);
  GValue pad_value = G_VALUE_INIT;

  while (gst_iterator_next(iter, &pad_value) == GST_ITERATOR_OK) {
    GstPad *pad = GST_PAD(g_value_get_object(&pad_value));
    const gchar *pad_name = gst_pad_get_name(pad);
    print_caps(prefix + "/" + pad_name, pad);
    g_free(const_cast<gchar *>(pad_name));
  }

  gst_iterator_free(iter);
}

void print_all_sink_pad_caps(const std::string & prefix, GstElement *element)
{
  if (!element) {
    return;
  }

  GstIterator *iter = gst_element_iterate_sink_pads(element);
  GValue pad_value = G_VALUE_INIT;

  while (gst_iterator_next(iter, &pad_value) == GST_ITERATOR_OK) {
    GstPad *pad = GST_PAD(g_value_get_object(&pad_value));
    const gchar *pad_name = gst_pad_get_name(pad);
    print_caps(prefix + "/" + pad_name, pad);
    g_free(const_cast<gchar *>(pad_name));
  }

  gst_iterator_free(iter);
}

void print_caps(const std::string & prefix, GstElement *element)
{
  if (!element) {
    return;
  }

  print_all_src_pad_caps(prefix, element);
  print_all_sink_pad_caps(prefix, element);
}

void print_caps(const std::string & prefix, GstElement *element, const std::string & pad_name)
{
  if (!element) {
    return;
  }

  GstPad *pad = gst_element_get_static_pad(element, pad_name.c_str());
  if (pad) {
    print_caps(prefix, pad);
  } else {
    std::cout << "no static pad named " << pad_name << std::endl;
  }
}

void print_caps(const std::string & prefix, GstPad *pad)
{
  if (!pad) {
    return;
  }

  const GstCaps *caps = gst_pad_get_current_caps(pad);

  if (!caps) {
    std::cout << prefix << ", no caps" << std::endl;
    return;
  }

  guint num_caps = gst_caps_get_size(caps);
  gboolean fixed = gst_caps_is_fixed(caps);
  std::cout << prefix << ", num_caps: " << num_caps << ", fixed: " << fixed << std::endl;

  for (guint i = 0; i < num_caps; ++i) {
    GstStructure *structure = gst_caps_get_structure(caps, i);

    gint width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    const GValue *framerate = gst_structure_get_value(structure, "framerate");
    auto framerate_str = gst_value_serialize(framerate);

    std::cout << prefix << "/" << i << ", width: " << width << ", height: " << height
      << ", framerate: " << framerate_str << std::endl;
  }
}

}  // namespace gst_util


