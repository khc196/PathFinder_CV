import cv2
import numpy as np
# 34 68 21
def nothing(x) :
    pass
def main():
    cv2.namedWindow('PID_K_Controller')
    cv2.resizeWindow('PID_K_Controller', 640,240)
    fr = open('pid_K', 'r')
    lines = fr.readlines()
    cv2.createTrackbar('KP', 'PID_K_Controller', int(lines[0]), 1000, nothing)
    cv2.createTrackbar('KI', 'PID_K_Controller', int(lines[1]), 1000, nothing)
    cv2.createTrackbar('KD', 'PID_K_Controller', int(lines[2]), 1000, nothing)

    while True:
        KP = cv2.getTrackbarPos('KP', 'PID_K_Controller')
        KI = cv2.getTrackbarPos('KI', 'PID_K_Controller')
        KD = cv2.getTrackbarPos('KD', 'PID_K_Controller')
        
        f = open('pid_K', 'w')
        f.write(str(KP))
        f.write('\n')
        f.write(str(KI))
        f.write('\n')
        f.write(str(KD))
        f.write('\n')
        
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main() 