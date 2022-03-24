import cv2
import numpy as np

# global variables go here:
counter = 0
hsv_min = (69, 107, 15)
hsv_max = (91, 255, 255)
area_max_pct = 0.05
area_min_pct = 0.0015
aspect_min = 1
aspect_max = 7

# To change a global variable inside a function,
# re-declare it the global keyword
def incrementTestVar():
    global counter
    counter = counter + 1
    if counter >= 250:
        print("tick")
        counter = 0

def drawDecorations(image):
    cv2.putText(image, 
        'Go Omegabytes!', 
        (0, 230), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .5, (0, 255, 0), 1, cv2.LINE_AA)
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    img_height = image.shape[0]
    img_width = image.shape[1]
    img_area = (img_height * img_width)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, hsv_min, hsv_max)
   
    contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        #cv2.drawContours(image, contours, -1, (255,0,255), 1)
        idx = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            area_pct = 100.0 * area / img_area
            if (area_pct > area_max_pct or area_pct < area_min_pct):
                continue
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            box = np.intp(cv2.boxPoints(rect))
            boxArea = cv2.contourArea(box)
            if boxArea <= 0:
                continue
            #cv2.drawContours(image,[box],0,(255,0,0),1)
            if h > w:
                # Adjust rectangle to standard configuration
                angle = angle + 90
                w, h = h, w
                box = np.array([box[1], box[2], box[3], box[0]])
            aspect = w / h
            if aspect < aspect_min or aspect > aspect_max:
                continue
            box_mask = np.zeros_like(img_threshold)
            cv2.drawContours(box_mask, [box], -1, 255, cv2.FILLED)
            masked_box = img_threshold & box_mask
            filled = np.count_nonzero(masked_box)
            filled_pct = 100.0 * filled / boxArea
            if counter == 0:
                print(filled_pct)
            #mask_area = sum(map(cv2.contourArea, [cornerLL, cornerLR, middle]))
            cv2.drawContours(image,[box],0,(255,0,0),1)
            idx = idx + 1
        if counter == 0:
            print("Count {}".format(idx))

        largestContour = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largestContour)

        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),1)
        llpython = [1,x,y,w,h,9,8,7]  
  
    incrementTestVar()
    drawDecorations(image)
       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, image, llpython