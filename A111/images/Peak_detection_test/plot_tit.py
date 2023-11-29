import matplotlib.pyplot as plt

img = plt.imread("A111/images/Peak_detection_test/peakDetectionP1.png")
plt.imshow(img)
plt.title("Peak detection")
plt.xlabel("Depth (mm)")
plt.ylabel("Amplitude (V)")
plt.show()
