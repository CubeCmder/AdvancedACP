import cv2
import time

# open video0
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Turn off auto exposure
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
# set exposure time
#cap.set(cv2.CAP_PROP_EXPOSURE, 0)

w=3264
h=2448
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FPS, 15)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    framer = cv2.resize(frame, (int(3264/2), int(2448/2)))
    # Display the resulting frame
    cv2.imshow('frame', framer)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        frame = cv2.fastNlMeansDenoisingColored(frame, None, 10, 10, 7, 21)
        cv2.imwrite("frame2.jpg", frame)
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()