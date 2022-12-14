

#!/usr/bin/python3
import sys
import cv2 as cv
import numpy as np
from picamera2 import Picamera2, Preview
import datetime
def main():
    
    # Grab images as numpy arrays and leave everything else to OpenCV.
    cv.startWindowThread()
    
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration()
    picam2.configure(camera_config)
    picam2.start()
    
    #time_start = datetime.datetime.now()
    all_circle=[]#[x,y,r,id,frame]
    id_count=0
    while True:
        

        src = picam2.capture_array()
        
        gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
         
        #gray = cv.GaussianBlur(gray, (5,5),0)
        cv.imshow("gray image", gray)

        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 20,
                                   param1=100, param2=30,
                                   minRadius=1, maxRadius=50)
        print(circles) 
        print(all_circle)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv.circle(src, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv.circle(src, center, radius, (255, 0, 255), 3)
                for idx,j in enumerate(all_circle):
                    print(i,j)
                    move=((int(j[0])-int(i[0]))**2)+((int(j[1])-int(i[1]))**2)
                    print(move)
                    if move>400:
                        continue
                    all_circle[idx][0],all_circle[idx][1],all_circle[idx][2], all_circle[idx][4]=i[0],i[1],i[2],0
                    new_id=j[3]
                    break
                else:
                    all_circle.append([i[0],i[1],i[2],id_count,0])
                    new_id=id_count
                    id_count+=1
                    
                cv.putText(src,str(new_id),center,cv.FONT_HERSHEY_TRIPLEX,1,(0,255,0),2)
            temp=[]
            for i in range(len(all_circle)):
                all_circle[i][4]+=1
                if all_circle[i][4]<30:
                    temp.append(all_circle[i])

            all_circle=temp

        cv.imshow("detected circles", src)
        cv.waitKey(1)
    
    #time_elapsed = (datetime.datetime.now()-time_start)
    #print(time_elapsed/100)
    
    return 0 
if __name__ == "__main__":
    main()
