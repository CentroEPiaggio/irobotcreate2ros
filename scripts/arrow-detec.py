import cv2

cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()
    cv2.imshow('Annotated Feed', frame)

    c = cv2.waitKey(1) 
    if c == 27: # this is the esc key
        break

cap.release()
cv2.destroyAllWindows()