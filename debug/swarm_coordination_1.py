# Import necessary libraries
import json
import time
from threading import Thread
from digi.xbee.devices import XBeeDevice

# ----------------------------
# Swarm Configuration
NO_OF_DRONES = 2
DRONE_NO = 1
XBEE_PORT = 'COM10'
BAUD_RATE = 9600
XBEES_NODE_IDs = ["Hexa1", "Hexa2"]
DISABLE_SWARM_COMMUNICATION = False
# ----------------------------

# Colour of the marker
MARKER_COLOUR = "red"
# Information about the detected markers. Key corresponds to marker colour and value its GPS coord.
DETECTED_MARKERS = {"red":"Ss"}


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


def main():
    # Instantiate the XBee device object
    device = XBeeDevice(XBEE_PORT, BAUD_RATE)

    # Get a list of all the drones that are online
    online_drones = get_remote_xbees(device)

    global DISABLE_SWARM_COMMUNICATION
    # Intialize the thread that will be used to listen for marker information
    receiver = Thread(target=receive_marker_info, args=[device, ], daemon=True)
    # Intialize the thread that will be used to send  marker information
    sender = Thread(target=send_marker_info, args=[device, online_drones], daemon=True)

    receiver.start()
    sender.start()

    time.sleep(50)
    DISABLE_SWARM_COMMUNICATION=True

    # Close the device object
    device.close()


if __name__ == "__main__":
    main()
