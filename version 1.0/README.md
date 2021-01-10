+ 1) Preprocessing: GaussianBlur --> Mask according to color(blue) --> Morphological operation (Close)
+ 2) Find contours and the hierarchies.
+ 3) Bubble sorted the contour and hierarchy pairs. The pairs needed to be in order from smallest to biggest according to the contour area (to ensure
the contour of the armor module (the small rectangle) will always get added to before searching for the contour of the arrow panel.)
+ 4) Append all the armor modules to a list (one of them will be associate with the arrow panel)
+ 5) Find the contour of the arrow panel according to the area (half the size of the big panel)
+ 6) Match the desired armor module with the arrow panel according to the hierarchical relationship.
+ 7) Feed the center coordinate of the armor module to the Kalman Filter for prediction.

Additionally, the program will take the first frame for template matching to select the region of interest (ROI)(focus on the power rune only), which can increase the efficiency.

Possible improvements: Get the precise measurement of the robot and the shooting environment for better parameter-tuning.
