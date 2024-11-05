# Import necessary libraries
import os
import cv2
import time
import math
import json
import numpy as np
from threading import Thread
from digi.xbee.devices import XBeeDevice
from dronekit import connect, Command, mavutil, VehicleMode

# ----------------------------
# Mission Configuration
PORT = '/dev/ttyACM0'
TAKE_OFF_ALTITUDE = 10  # m
SAVE_SEARCH_MISSION = True
EXECUTE_SAR_MISSION = False
# ----------------------------
# Marker Search Configuration
MARKER_COLOUR = "red"
CAMERA_INDEX = 0
OUTPUT_VIDEO_NAME = "camera_feed.avi"
DISABLE_CAMERA_FEED = False
SHOW_CAMERA_FEED = False
PAYLOAD_SERVO_NO = 9
PAYLOAD_SERVO_PWM = 1100
# ----------------------------
# Swarm Configuration
NO_OF_DRONES = 2
DRONE_NO = 1
XBEE_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
XBEES_NODE_IDs = ["Hexa1", "Hexa2"]
DISABLE_SWARM_COMMUNICATION = False
# ----------------------------


# Information about the detected markers. Key corresponds to marker colour and value its GPS coord.
DETECTED_MARKERS = {}


def arm_and_takeoff(vehicle, altitude):
    """
    This function will arm the vehicle and fly it to mission altitude
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(alt=altitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached mission altitude (" + str(altitude) + "m)")
            break
        time.sleep(1)


def search_mission(vehicle, survey_coords):
    """
    This function will generate a survey mission and upload it to the vehicle.
    """
    print("Generating the survey mission")
    cmds = vehicle.commands

    while not vehicle.home_location:
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print("Waiting for home location...")

    cmds.clear()
    home = vehicle.location.global_frame
    vehicle.home_location = home

    # Take off  command
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                0, 0, 0, 0, TAKE_OFF_ALTITUDE))

    for coord in survey_coords:
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                    0, 0, 0, 0, 0, coord[0], coord[1], TAKE_OFF_ALTITUDE))

    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                0, 0, 0, 0, 0, survey_coords[-1][0], survey_coords[-1][1], TAKE_OFF_ALTITUDE - 2))

    print("Mission generation complete")

    cmds.upload()
    print("Search mission uploaded successfully!")
    return cmds


def rescue_mission(cmds, marker_coords):
    """
        This function will generate a rescue mission and upload it to the vehicle.
    """
    # Clear previous commands
    cmds.clear()

    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                0, 0, 0, 0, 0, marker_coords.lat, marker_coords.lon, TAKE_OFF_ALTITUDE - 4))

    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, 0, PAYLOAD_SERVO_NO, PAYLOAD_SERVO_PWM, 0, 0, 0, 0, 0))

    cmds.upload()


def save_mission(cmds, dest_path: str):
    """
    Save a mission in the Waypoint file format.
    """
    print("Saving mission file to ", dest_path, " ...")
    output = 'QGC WPL 110\n'
    for cmd in cmds:
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            cmd.seq, cmd.current, cmd.frame, cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.x, cmd.y,
            cmd.z, cmd.autocontinue)
        output += commandline
    with open(dest_path, 'w') as file:
        file.write(output)


def get_remote_xbees(device):
    """
    This function will return all the XBees that are available on the same network.
    """
    # A dictionary that stores drone_id as key and corresponding XBee object as value
    remote_xbees = {}
    # Open the communication with the XBee and load information about it
    device.open()
    # Flushe the packets queue
    device.flush_queues()
    # Set the timeout in seconds for synchronous operations
    device.set_sync_ops_timeout(10)
    print("Searching for drones on the same network...")
    # Get all the remote XBees
    # Get all the destination XBees
    for drone_id in XBEES_NODE_IDs:
        if not XBEES_NODE_IDs[DRONE_NO - 1] == drone_id:
            xbee_network = device.get_network()
            destination_xbee = xbee_network.discover_device(drone_id)
            if destination_xbee is not None:
                remote_xbees[drone_id] = destination_xbee
    print("Drone that are online:", list(remote_xbees.keys()))
    return remote_xbees


def send_marker_info(device, online_drones):
    """
    This function will send the marker detection information to other drones in the swarm
    """
    # Convert dictionary to bytes so that it can be sent
    detections_info = json.dumps(DETECTED_MARKERS).encode('utf-8')
    while not DISABLE_SWARM_COMMUNICATION:
        for drone in online_drones.keys():
            print("Sending detected marker info to", drone)
            # Send data to the destination drone asynchronously
            device.send_data_async(online_drones[drone], detections_info)
        time.sleep(5)


def receive_marker_info(device):
    """
    This function will receive the marker detection information from other drones in the swarm
    """
    while not DISABLE_SWARM_COMMUNICATION:
        # Read new data received
        received_message = device.read_data()
        if received_message is not None:
            print("Received detection information from other drones in the swarm")
            detections_from_swarm = json.loads(received_message.data.decode('utf-8'))
            for marker_colour in detections_from_swarm.keys():
                if marker_colour == MARKER_COLOUR and len(detections_from_swarm[marker_colour]) > 0:
                    print("Coordinates of " + marker_colour + " marker received")
                    DETECTED_MARKERS[MARKER_COLOUR] = detections_from_swarm[marker_colour]


def marker_search(vehicle):
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
        nearest_marker_gps_coord = vehicle.location.global_frame

        marker_detections[marker_colour].append((nearest_marker_distance, nearest_marker_gps_coord))

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
    writer = cv2.VideoWriter('camera_feeds/' + OUTPUT_VIDEO_NAME, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 5,
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


def main():
    # Connect to the vehicle
    vehicle = connect(PORT, wait_ready=True, baud=57600)

    # Instantiate the XBee device object
    device = XBeeDevice(XBEE_PORT, BAUD_RATE)

    # Get a list of all the drones that are online
    online_drones = get_remote_xbees(device)

    global DISABLE_CAMERA_FEED
    # Intialize the thread that will be used identify markers on ground
    marker_search_thread = Thread(target=marker_search, args=[vehicle, ], daemon=True)

    global DISABLE_SWARM_COMMUNICATION
    # Intialize the thread that will be used to listen for marker information
    receiver = Thread(target=receive_marker_info, args=[device, ], daemon=True)
    # Intialize the thread that will be used to send  marker information
    sender = Thread(target=send_marker_info, args=[device, online_drones], daemon=True)

    # Check that vehicle is armable. This ensures home_location is set.
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.groundspeed = 2  # m/s

    survey_coords = [[24.89007772334218, 67.09057360035047],
                     [24.89007772334218, 67.09057360035047],
                     [24.89007772334218, 67.09066939352957],
                     [24.890170180239828, 67.09066939352957],
                     [24.89007772334218, 67.09076518670867],
                     [24.89007772334218, 67.09076518670867]]

    cmds = search_mission(vehicle=vehicle, survey_coords=survey_coords)

    if SAVE_SEARCH_MISSION:
        # Save the mission in waypoint file format
        directory = "missions"
        if not os.path.exists(directory):
            os.makedirs(directory)
        mission_file_path = directory + '/' + 'drone.plan'
        save_mission(cmds=cmds, dest_path=mission_file_path)

    if EXECUTE_SAR_MISSION:

        # Start the marker search
        marker_search_thread.start()
        time.sleep(5)
        print("Marker search started!")

        # Start listening for information from other drones
        receiver.start()

        # Start sending information to other drones
        sender.start()

        # Arm the drone and fly it to mission altitude
        arm_and_takeoff(vehicle, TAKE_OFF_ALTITUDE)

        vehicle.mode = VehicleMode("AUTO")

        print("Search in progress... ")
        while True:
            if vehicle.location.global_relative_frame.alt <= TAKE_OFF_ALTITUDE - 2:
                print("Search finished!")

                # Disable the camera feed
                DISABLE_CAMERA_FEED = True

                # Get outside the loop
                break

        # Get the coordinates of the relevant marker if they are found
        if DETECTED_MARKERS:
            print("Mode cahnged to LOITER")
            vehicle.mode = VehicleMode("LOITER")

            print("Starting the rescue mission")
            marker_coords = DETECTED_MARKERS[MARKER_COLOUR]
            rescue_mission(marker_coords=marker_coords, cmds=cmds)

            vehicle.mode = VehicleMode("AUTO")

            while True:
                if vehicle.location.global_relative_frame.alt <= TAKE_OFF_ALTITUDE - 4:
                    time.sleep(3)
                    print("Rescue finished!")
                    # Get outside the loop
                    break

        vehicle.mode = VehicleMode("LAND")

    # Close the vehicle object
    vehicle.close()


if __name__ == "__main__":
    main()
