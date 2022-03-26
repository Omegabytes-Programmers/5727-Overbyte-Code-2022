#!/usr/bin/python3

import sys

# vvv CUT HERE FOR LIMELIGHT vvv ###

import cv2
import numpy as np
import operator

interactive = 0
tick_count = 0
counter = 0
#hsv_min = (55, 120, 105) #ncgui-0
#hsv_max = (92, 215, 255) #ncgui-0
#hsv_min = (69, 107, 15) #llp tuning
#hsv_max = (99, 255, 255) #llp tuning
hsv_min = (55, 107, 15)
hsv_max = (99, 215, 255)
area_max_pct = 0.1000 # 0.05
area_min_pct = 0.0015 # 0.005
aspect_min = 1 # 0.17
aspect_max = 7 # 3.05
min_fill_pct = 20 # 10
morph_op = cv2.MORPH_CLOSE
morph_kernel_size = 4
xdiff_min = 1.3
xdiff_max = 2.5
ydiff_max = 3.5

colors = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'magenta': (255, 0, 255),
    'cyan': (255, 255, 0),
    'white': (255, 255, 255),
    'gray': (128, 128, 128)
}

# runPipeline() is called every frame by Limelight's backend
def runPipeline(image, llrobot):
    global counter
    
    # Get image size
    img_height = image.shape[0]
    img_width = image.shape[1]
    img_area = (img_height * img_width)
    
    # Filter image by color
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, hsv_min, hsv_max)
    if interactive:
        thresh_big = cv2.resize(img_threshold, None, fx = 2, fy = 2)
        #cv2.imshow("threshold", thresh_big)
    
    # Dilate and erode to smooth out image
    kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    img_morph = cv2.morphologyEx(img_threshold, morph_op, kernel)
    if interactive:
        morph_big = cv2.resize(img_morph, None, fx = 2, fy = 2)
        cv2.imshow("morph", morph_big)
   
    # Find contours (groups of pixels)
    contours, _ = cv2.findContours(img_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    finalContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]
    best = None
    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, colors['magenta'], 1)
        good_contours = []
        for cnt in contours:
            M = cv2.moments(cnt)
            area = M['m00']
            if area == 0:
                continue
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            # Ensure that the contour is within the size bounds
            area_pct = 100.0 * area / img_area
            if (not area_min_pct < area_pct < area_max_pct):
                if counter == 0:
                    print("Rejected contour with {:4f} percent area".format(area_pct))
                if area_pct > 0:
                    cv2.drawContours(image, [cnt], 0, colors['red'], 1)
                continue
            
            # Find the minimum bounding rectangle (could be angled)
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            box = np.intp(cv2.boxPoints(rect))
            boxArea = cv2.contourArea(box)
            
            # Reject anything without area
            if boxArea == 0:
                continue
            
            # Adjust rectangle to standard configuration
            # TODO Revisit -- snap 43
            if h > w:
                angle = angle + 90
                w, h = h, w
                box = np.array([box[1], box[2], box[3], box[0]])

            # Filter by the aspect ratio
            aspect = w / h
            if interactive:
                print("Aspect ratio: {:4f}".format(aspect))
            if not aspect_min < aspect < aspect_max:
                if counter == 0:
                    print("Rejecting based on aspect ratio: {:4f}".format(aspect))
                cv2.drawContours(image, [box], 0, colors['cyan'], 1)
                continue

            # Filter by the amount the contour fills the bounding box
            box_mask = np.zeros_like(img_threshold)
            cv2.drawContours(box_mask, [box], -1, 255, cv2.FILLED)
            masked_box = img_threshold & box_mask
            filled = np.count_nonzero(masked_box)
            filled_pct = 100.0 * filled / boxArea
            if counter == 0:
                print("Filled {:2f}".format(filled_pct))
            if filled_pct < min_fill_pct:
                cv2.drawContours(image, [box], 0, colors['gray'], 1)
                continue

            good_contours.append([cnt, box, cx, cy, w, h])
            
        if counter == 0:
            print("Good contours: {}".format(len(good_contours)))
            
        good_sorted = sorted(good_contours, key=operator.itemgetter(2))
        left_sorted = []
        for cnt_info in good_sorted:
            (cnt, box, cx, cy, w, h) = cnt_info
            cv2.drawContours(image, [box], 0, colors['blue'], 1)
            if counter == 0:
                print("Good: {} x {} @ {}, {}".format(w, h, cx, cy))
            left = None
            # Look at all other contours
            for other in reversed(left_sorted):
                xdiff = (cx - other[2]) / w
                xdiff2 = (cx - other[2]) / other[4]
                ydiff = abs(cy - other[3]) / h
                if counter == 0:
                    print("DIFF = {:2.2f} / {:2.2f}, {:2.2f}".format(xdiff, xdiff2, ydiff))
                if xdiff < xdiff_min and xdiff2 < xdiff_min:
                    continue
                if xdiff > xdiff_max and xdiff2 > xdiff_max:
                    break
                if ydiff > ydiff_max:
                    continue
                left = other # TODO REmove
                            
                # Found a target to the left 
                if counter == 0:
                    print("Found left @ {}, {}".format(left[2], left[3]))
                
                # Record best match it nothing found yet
                if best is None:
                    if counter == 0:
                        print("New best")
                    best = [left, cnt_info]
                    break
                
                # Add to the best match if this is a continuation
                elif best[-1][2] == left[2] and best[-1][3] == left[3]:
                    if counter == 0:
                        print("Added to best")
                    best.append(cnt_info)
                    break
                
                # Consider if this can replace the best match (should be rare)
                elif len(best) <= 2 and cy < best[-1][3]:
                    if counter == 0:
                        print("Replaced best")
                    best = [left, cnt_info]
                    break
                    
                elif counter == 0:
                    print("Not using")
            
            # Add counter to list of those processed and on the left
            left_sorted.append(cnt_info)    

    if best is not None:
        if counter == 0:
            print("Best count: {}".format(len(best)))
               
        # Build a hull around all items in the best match
        list_of_pts = [] 
        for ctr_info in best:
            list_of_pts += [pt[0] for pt in ctr_info[0]]
        ctr = np.array(list_of_pts).reshape((-1,1,2)).astype(np.int32)
        hull = cv2.convexHull(ctr)
        cv2.drawContours(image, [hull], -1, colors['yellow'], 1)
        
        xavg = int(sum(map(operator.itemgetter(2), best)) / len(best))
        ytop = min(map(operator.itemgetter(3), best))
        if interactive:
            cv2.circle(image, (xavg, ytop), 3, colors['green'], -1)
        
        finalContour = hull
        finalContour = np.array([[xavg,ytop]]).reshape((-1,1,2)).astype(np.int32)
    else:
        cv2.putText(image, 'No Target!', (0, 230), cv2.FONT_HERSHEY_SIMPLEX, .5, colors['red'], 1, cv2.LINE_AA)

    if not interactive:
        counter = counter + 1
        if tick_count > 0 and counter >= tick_count:
            print("=== TICK ===")
            counter = 0
        
    return finalContour, image, llpython

# ^^^ CUT HERE FOR LIMELIGHT ^^^ ###

# MAIN
if __name__=="__main__":
    interactive = 1
    first = True
    for fn in sys.argv:
        if first:
            first = False
            continue
        img = cv2.imread(fn)
        img = cv2.rotate(img, cv2.ROTATE_180)
        print("=== Running pipeline on {}".format(fn))
        (primary, img_out, llp) = runPipeline(img, [])
        resized = cv2.resize(img_out, None, fx = 2, fy = 2)
        cv2.imshow(fn, resized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()