import cv2
'''
apiPreference    preferred Capture API backends to use. 
Can be used to enforce a specific reader implementation 
if multiple are available: 
e.g. cv2.CAP_MSMF or cv2.CAP_DSHOW.
'''
# open video0
cap = cv2.VideoCapture(0, cv2.CAP_MSMF)
# set width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3264)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2448)
## Turn off auto exposure
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
## set exposure time
#cap.set(cv2.CAP_PROP_EXPOSURE, -6)
# set fps
# cap.set(cv2.CAP_PROP_FPS, 30)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()