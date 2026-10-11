## WORK IN PROGRESS ##

import cv2
import numpy as np
import matplotlib.pyplot as plt


def __calculate_channel_histogram__(img, channel, bins=256):
    hist = cv2.calcHist([img], [channel], None, [bins], [0, 256])
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    return hist

def __calculate_RGB_histogram__(img, bins=256, channels=[2,1,0]):  # channel order in opencv is BGR
    hists = []
    for channel in channels:
        hists.append(__calculate_channel_histogram__(img, channel, bins)) 
    return hists

def __compare_channel_histograms__(hist1, hist2):
    diff = cv2.compareHist(hist1, hist2, cv2.HISTCMP_BHATTACHARYYA)
    return diff

def __compare_RGB_histograms__(hists1, hists2):
    diffs = []
    for channel in range(len(hists1)):
        diffs.append(__compare_channel_histograms__(hists1[channel], hists2[channel]))
    return diffs


# histogram
bins = 256
channels = [2, 1, 0]

# awb
threshold = 0.05
awbgains = [1, 1]

max_iterations = 100

path1 = "images/tmp/awb.jpg"
img1 = cv2.imread(path1)
hists1 = __calculate_RGB_histogram__(img1, bins=bins, channels=channels)

# Remove the first and last bins
hists1 = [hist[1:-1] for hist in hists1]
cv2.imshow("img1", img1)

path2 = "images/tmp/img0.jpg"
img2 = cv2.imread(path2)

for _ in range(max_iterations):
    img2_copy = img2.copy()
    img2_copy[:,:,2] = np.clip(img2_copy[:,:,2] * awbgains[0], 0, 255)
    img2_copy[:,:,0] = np.clip(img2_copy[:,:,0] * awbgains[1], 0, 255)
    hists2 = __calculate_RGB_histogram__(img2_copy, bins=bins, channels=channels)

    hists2 = [hist[1:-1] for hist in hists2]

    cv2.imshow("img2", img2_copy)

    diffs = __compare_RGB_histograms__(hists1, hists2)
    print("diff:", round(diffs[0],2), round(diffs[1],2), round(diffs[2],2))

    # Display histograms
    plt.figure(figsize=(18, 6))
    plt.subplot(1, 3, 1)
    plt.hist(hists1[0].ravel(), bins=bins, color='r', alpha=0.3)
    plt.hist(hists2[0].ravel(), bins=bins, color='r', alpha=1)
    plt.title('Red Channel')

    plt.subplot(1, 3, 2)
    plt.hist(hists1[1].ravel(), bins=bins, color='g', alpha=0.3)
    plt.hist(hists2[1].ravel(), bins=bins, color='g', alpha=1)
    plt.title('Green Channel')

    plt.subplot(1, 3, 3)
    plt.hist(hists1[2].ravel(), bins=bins, color='b', alpha=0.3)
    plt.hist(hists2[2].ravel(), bins=bins, color='b', alpha=1)
    plt.title('Blue Channel')
    plt.show()

    # Red
    print("R median", np.median(hists1[0]), np.median(hists2[0]))
    print("R max", np.max(hists1[0]), np.max(hists2[0]))
    if diffs[0] > threshold:
        awbgains[0] *= 1 + 0.01 * (np.median(hists1[0]) - np.median(hists2[0]))

    print("G median", np.median(hists1[1]), np.median(hists2[1]))
    print("G max", np.max(hists1[1]), np.max(hists2[1]))

    # Blue
    print("B median", np.median(hists1[2]), np.median(hists2[2]))
    if diffs[2] > threshold:
        awbgains[1] *= 1 + 0.01 * (np.median(hists1[2]) - np.median(hists2[2]))

    print("awbgains:", awbgains)

    if diffs[0] <= threshold and diffs[2] <= threshold:
        break

    cv2.waitKey(100)
cv2.waitKey(0)

          
