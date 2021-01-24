std_msgs/Header header

float64 dt                      # Delta time from previous stereo frame

int32[2] features               # Number of features in each image, 0=left, 1=right
int32 features_match_raw        # Match (any distance)
int32 features_match            # Match (< max distance)
int32 features_match_time_raw   # Left image matches across time (any distance)
int32 features_match_time       # Left image matches across time (< max distance)
int32 features_match_4          # Match across all 4 images

float64 median_epipolar_error   # Median epipolar error
float64 median_disparity        # Median disparity difference
float64 median_distance         # Median match distance

int32[2] time_image_init        # Time to init image (find features, etc.), 0=left, 1=right, us
int32 time_stereo_init          # Time to init stereo image, left + right + more, us
int32 time_compute_tf           # Time to compute transform, us
int32 time_callback             # Time to process stereo image, stereo_init + compute_tf + more, us