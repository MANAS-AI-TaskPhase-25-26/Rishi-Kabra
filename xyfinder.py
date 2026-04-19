import cv2

img = cv2.imread('costmap_visual.png')
if img is None:
    print("Could not find the image file")
    exit()

img_display = img.copy()

clicked_points =[]


def click_event(event, x, y, flags, params):
    global img_display
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"point = ({x}, {y})")
        clicked_points.append((x, y))

        cv2.drawMarker(img_display, (x, y), (0, 255, 0), 
                       markerType=cv2.MARKER_CROSS, markerSize=10, thickness=4)
        
        cv2.putText(img_display, f"({x},{y})", (x + 5, y - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow('Point Selector', img_display)

        if len(clicked_points) == 2:
            save_points()


def save_points():
    p1, p2 = clicked_points[0], clicked_points[1]
    data_string = f"{p1[0]},{p1[1]},{p2[0]},{p2[1]}"
    with open('pathPoints.txt', 'w') as f:
        f.write(data_string)

cv2.namedWindow('Point Selector', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Point Selector', 1200, 800)
cv2.setMouseCallback('Point Selector', click_event)

cv2.imshow('Point Selector', img_display)
cv2.waitKey(0)
cv2.destroyAllWindows()