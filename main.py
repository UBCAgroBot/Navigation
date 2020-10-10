import cv2
import numpy as np
import hough_algorithm

vid = cv2.VideoCapture("cropVid.mp4")
 
if(vid.isOpened() == False):
    print("Error Opening Video File")


while(vid.isOpened()):
    ret, frame = vid.read()
    if ret == False:
        print("No More Frames Remaining")
        break
    
    ################### ADD ALGORITHM HERE ###################
    h = hough_algorithm.hough_algorithm()
    h.process_frame(frame)


    key = cv2.waitKey(25)
    # If Enter is pressed capture current frames
    if key == 13:
        # Save Current Frames and Exit
        cv2.imwrite('original.png',frame)
        res = cv2.bitwise_and(frame,frame, mask= mask)
        cv2.imwrite('colour_mask.png',mask)
        cv2.imwrite('colour_res.png',res)
        cv2.imwrite('edges.png',edges)
        cv2.imwrite('lineimg.png',lineimg)
    # Exit if Esc key is pressed
    if key == 27:
        break
 
vid.release()
cv2.destroyAllWindows()