import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized

def color_hist(img, nbins=32, bins_range=(0,256)):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h_hist = np.histogram(hsv_img[:,:,0], bins=nbins, range=bins_range)
    s_hist = np.histogram(hsv_img[:,:,1], bins=nbins, range=bins_range)
    v_hist = np.histogram(hsv_img[:,:,2], bins=nbins, range=bins_range)
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
    hsum = np.sum(hist_features)
    norm_features = hist_features / hsum if hsum > 1e-8 else hist_features
    return norm_features

def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # Compute histograms
    ch1_hist = np.histogram(channel_1_vals, bins=32, range=(0,256))
    ch2_hist = np.histogram(channel_2_vals, bins=32, range=(0,256))
    ch3_hist = np.histogram(channel_3_vals, bins=32, range=(0,256))
    
    # Concatenate and normalize the histograms
    features = np.concatenate((ch1_hist[0], ch2_hist[0], ch3_hist[0])).astype(np.float32)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    #normed_features = np.random.random(96)
    fsum = np.sum(features)
    norm_features = features / fsum if fsum > 1e-8 else features
    return norm_features


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # Compute histograms of normal values (just like with color)

    # Concatenate and normalize the histograms
    nx_hist = np.histogram(norm_x_vals, bins=32, range=(0,256))
    ny_hist = np.histogram(norm_y_vals, bins=32, range=(0,256))
    nz_hist = np.histogram(norm_z_vals, bins=32, range=(0,256))

    # Concatenate and normalize the histograms
    features = np.concatenate((nx_hist[0], ny_hist[0], nz_hist[0])).astype(np.float32)
    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    fsum = np.sum(features)
    norm_features = features / fsum if fsum > 1e-8 else features
    return norm_features
