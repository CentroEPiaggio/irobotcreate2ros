import apriltag
import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
import math

def arrow_outline(img):
    dst = cv2.Canny(img, 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10) #probabilistic line transform
    final_pts = []
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            
            thresh = 30
     
            if abs(l[1]-l[3]) < thresh: # aka vertical line
                pass
            elif abs(l[0]-l[2]) < thresh: # aka horizontal line
                pass
            else:
                start_pt = (l[0], l[1])
                end_pt = (l[2], l[3])
                final_pts.append(start_pt)
                final_pts.append(end_pt)
                cv2.line(img, start_pt, end_pt, (0,0,255), 3, cv2.LINE_AA)
    
    return img, final_pts


def arrow_direc(img, arrow_pts):
    
    if not arrow_pts:
        return 'NONE'
    
    else:
        arrow_coords = np.ravel(arrow_pts)
        x_list = arrow_coords[::2]
        y_list = arrow_coords[1::2]

        xmax = np.max(x_list)
        xmin = np.min(x_list)
        ymax = np.max(y_list)
        ymin = np.min(y_list)

        if (xmin, ymax) in arrow_pts:
            return 'RIGHT'

        else:
            return 'LEFT' 

def blue_detec(imageFrame):
    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value)
    # color space
    blur_val = 11 # should be positive and odd
    imageFrame = cv2.GaussianBlur(imageFrame, (blur_val,blur_val), 0)
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
  
    # Set range for blue color and
    # define mask
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
      
    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernel = np.ones((5, 5), "uint8")
      
    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernel)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask = blue_mask)

  
    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)[-2:]
    box_dims = []
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
            box_dims.append((x, y, w, h))

    return imageFrame, box_dims


def aprildec(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    print("[INFO] detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] tag family: {}".format(tagFamily))

def main():
    cap = cv2.VideoCapture(2)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        ret, frame = cap.read()
        
        # Arrow detection
        blue_area, blue_bbox_dims = blue_detec(frame)
        
        if len(blue_bbox_dims) != 0:
            blue = True
        else: 
            blue = False
            
        arrow_finding, arrow_pts = arrow_outline(frame)
        direc = arrow_direc(arrow_finding, arrow_pts)
        
        org = (50, 50)        
        if blue is True and direc !='NONE' and direc !='IDK':
            cv2.putText(arrow_finding, direc, org, cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1,color = (0, 0, 0), 
                        thickness = 2 )
        
        # Apriltag detection
        taginfo = aprildec(frame) #returing nothing at the moment
        
        # Show camera feed
        cv2.imshow('Annotated Feed', frame)
                
        c = cv2.waitKey(1) # adds delay in feed so we can read 
        if c == 27: # this is the esc key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()