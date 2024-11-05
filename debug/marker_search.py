import os
import cv2
import time
import math
import numpy as np
from threading import Thread

# ----------------------------
# Marker Search Configuration
MARKER_COLOUR = "red"
CAMERA_INDEX = 0
OUTPUT_VIDEO_NAME = "camera_feed.avi"
DISABLE_CAMERA_FEED = False
SHOW_CAMERA_FEED = True
# ----------------------------

# Information about the detected markers. Key corresponds to marker colour and value its GPS coord.
DETECTED_MARKERS = {}


def marker_search():
    """
    This function will identify markers placed on ground.
    """

    def detect_marker(image):
        """
        This function will detect markers from the frame if there are any.
        """
        img_area = image.shape[0] * image.shape[1]
        min_marker_area = img_area * 0.001
        max_marker_area = img_area * 0.9

        marker_coords = []
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            shape = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            if len(shape) == 4 and min_marker_area < area < max_marker_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
                marker_coords.append([x, y, w, h])
        return marker_coords

    def get_gps_coordinate(marker_colour: str, marker_coords: list):
        """
        This function will get the GPS coordinate of the nearest marker.
        """
        frame_center = [img_size[0] / 2, img_size[1] / 2]
        marker_distances = {}
        for marker_no in range(len(marker_coords)):
            # Calculate horizontal center of the marker (x0 + w/2)
            marker_center_x = marker_coords[marker_no][0] + (marker_coords[marker_no][2] / 2)
            # Calculates vertical center of box (y0 + h/2)
            marker_center_y = marker_coords[marker_no][1] + (marker_coords[marker_no][3] / 2)

            marker_center = [marker_center_x, marker_center_y]

            # Calculating the distance between marker center and frame center
            marker_distance = math.dist(marker_center, frame_center)
            marker_distances[marker_no] = marker_distance

        # Grab the marker nearest to frame center
        nearest_marker_distance = sorted(marker_distances.values())[0]
        # Get the gps coordinates
        # nearest_marker_gps_coord = vehicle.location.global_frame

        # marker_detections[marker_colour].append((nearest_marker_distance, nearest_marker_gps_coord))

    def update_gps_estimate(marker_colour: str):
        """
        This function will update the GPS coordinate estimate of the ground marker utilizing detections
        obtained from multiple frames
        """
        shortest_distance = 0
        markers = marker_detections[marker_colour]
        for marker_no in range(len(markers)):
            marker_distance, marker_gps_coords = markers[marker_no]
            if marker_no == 0:
                shortest_distance = marker_distance
                DETECTED_MARKERS[marker_colour] = marker_gps_coords
            if marker_distance < shortest_distance:
                shortest_distance = marker_distance
                DETECTED_MARKERS[marker_colour] = marker_gps_coords

        if DETECTED_MARKERS:
            cv2.putText(frame, "Marker Colour: " + marker_colour,
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                        1, cv2.LINE_AA, False)
            cv2.putText(frame, "GPS Coordinate: " + str(DETECTED_MARKERS[marker_colour]),
                        (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),
                        1, cv2.LINE_AA, False)

    # Create a VideoCapture object inorder to capture video from webcam
    cap = cv2.VideoCapture(CAMERA_INDEX)
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Unable to start the camera")
    # Get the default frame size
    img_size = (int(cap.get(3)), int(cap.get(4)))
    # Create camera_feeds directory if it does not exist
    if not os.path.exists("camera_feeds"):
        os.makedirs("camera_feeds")
    # Define the codec and create VideoWriter object
    writer = cv2.VideoWriter('../camera_feeds/' + OUTPUT_VIDEO_NAME, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 5,
                             img_size)
    # used to record the time when we processed last frame
    prev_frame_time = 0
    # Marker detections from all the processed frames. Each key contains a list of tuples as the value. First
    # element of tuple is the marker distance (relative to frame center) and second element is a tuple which has lat
    # and long of the marker
    marker_detections = {"red": [], "blue": []}

    while True:
        # Read frame from the camera
        ret, frame = cap.read()
        if ret:
            # Flip the original frame about y-axis
            frame = cv2.flip(frame, 1)

            # time when we finish processing for this frame
            new_frame_time = time.time()

            # Convert frame from BGR to HSV colour space
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define colour ranges
            lower_red_1 = np.array([159, 50, 70])
            upper_red_1 = np.array([180, 255, 255])
            lower_red_2 = np.array([0, 50, 70])
            upper_red_2 = np.array([9, 255, 255])
            lower_blue = np.array([90, 50, 70])
            upper_blue = np.array([128, 255, 255])

            # Get the masks
            red_mask_1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
            red_mask_2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
            blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

            # Detect the marker(s)
            red_marker_coords_1 = detect_marker(image=red_mask_1)
            red_marker_coords_2 = detect_marker(image=red_mask_2)
            red_marker_coords = red_marker_coords_1 + red_marker_coords_2
            blue_marker_coords = detect_marker(image=blue_mask)

            # Get the gps coordinate of the nearest marker
            if len(red_marker_coords) > 0:
                print("Red marker detected. Best estimate of its GPS coordinate has been saved!")
                get_gps_coordinate(marker_colour="red", marker_coords=red_marker_coords)
                update_gps_estimate(marker_colour="red")
            if len(blue_marker_coords) > 0:
                print("Blue marker detected. Best estimate of its GPS coordinate has been saved!")
                get_gps_coordinate(marker_colour="blue", marker_coords=blue_marker_coords)
                update_gps_estimate(marker_colour="blue")

            fps = str(int(1 / (new_frame_time - prev_frame_time)))
            prev_frame_time = new_frame_time
            print("fps: ", fps)

            # Write the frame
            writer.write(frame)

            if SHOW_CAMERA_FEED:
                # Display the resulting frame
                cv2.imshow('Camera Feed', frame)
                # Press Q on keyboard to stop the feed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if DISABLE_CAMERA_FEED:
                break

        # Break the loop
        else:
            break

    print("\nCleaning up...")
    # When everything done, release the video capture and video write objects
    cap.release()
    writer.release()
    # Destroy all the windows if they were created
    cv2.destroyAllWindows()
    print("Marker search ended")


if __name__ == "__main__":
    # Start the marker search in a separate thread
    marker_search = Thread(target=marker_search, daemon=True)
    marker_search.start()
    print("Marker search started!")

    input("Press the enter key to stop looking for markers...")
    END_CAMERA_FEED = True

    # Wait for everything to close properly
    time.sleep(1)
