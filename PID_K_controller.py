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
    cv2.createTrackbar('DI', 'PID_K_Controller', int(lines[3]), 100, nothing)
    cv2.createTrackbar('GB', 'PID_K_Controller', int(lines[4]), 100, nothing)
    cv2.createTrackbar('ST', 'PID_K_Controller', int(lines[5]), 30, nothing)
    while True:
        KP = cv2.getTrackbarPos('KP', 'PID_K_Controller')
        KI = cv2.getTrackbarPos('KI', 'PID_K_Controller')
        KD = cv2.getTrackbarPos('KD', 'PID_K_Controller')
        DI = cv2.getTrackbarPos('DI', 'PID_K_Controller')
        GB = cv2.getTrackbarPos('GB', 'PID_K_Controller')
        ST = cv2.getTrackbarPos('ST', 'PID_K_Controller')
        f = open('pid_K', 'w')
        f.write(str(KP))
        f.write('\n')
        f.write(str(KI))
        f.write('\n')
        f.write(str(KD))
        f.write('\n')
        f.write(str(DI))
        f.write('\n')
        f.write(str(GB))
        f.write('\n')
        f.write(str(ST))
        f.write('\n')
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main() 