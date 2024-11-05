# Import all the necessary modules
import time
import json
import geopandas
import numpy as np
import customtkinter
import tkintermapview
from shapely.ops import split
from PIL import Image, ImageTk
from dronekit_sitl import SITL
from shapely.geometry import LineString, Polygon, MultiPolygon
from dronekit import connect, LocationGlobal, Command, mavutil

# Set the appearance mode and color theme
customtkinter.set_appearance_mode("dark")
customtkinter.set_default_color_theme("dark-blue")


class App(customtkinter.CTk):
    """
    This class defines an app which can be used to select an appropriate search area for SAR mission.
    """
    APP_NAME = "SAR Planner - Team NUST AirWorks Beta"
    WIDTH = 1000
    HEIGHT = 700
    BUTTON_TEXT_FONT = "Calibri"
    HOME_NAME = "TEKNOFEST Flight Area"
    HOME_COORDS = "39.1131039,30.1371935"
    # For downloading offline map
    TOP_LEFT = (39.1341172, 30.0924685)
    BOTTON_RIGHT = (39.0941600, 30.1741793)
    MAP_DATABASE_PATH = "map_database/zafer_airport.db"
    # For saving
    PLAN_SAVE_PATH = "search_plan.json"
    GENERATE_SURVEY_MISSION = True
    MISSION_SAVE_PATH = 'missions/drone.plan'

    def __init__(self, *args, **kwargs):
        """
        Constructor for the App class.
        """
        super().__init__(*args, **kwargs)

        self.title(App.APP_NAME)
        self.pakistan_flag = ImageTk.PhotoImage(
            Image.open("media/pakistan-flag.png").resize((130, 100), Image.ANTIALIAS))
        self.naw_logo = ImageTk.PhotoImage(Image.open("media/naw-beta-logo.png").resize((130, 100), Image.ANTIALIAS))
        self.geometry(str(App.WIDTH) + "x" + str(App.HEIGHT))
        self.minsize(App.WIDTH, App.HEIGHT)

        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.bind("<Command-q>", self.on_closing)
        self.bind("<Command-w>", self.on_closing)
        self.createcommand('tk::mac::Quit', self.on_closing)

        self.marker_list = []
        self.search_area_coords = []
        self.path_list = []
        self.search_plan = []

        # ----- Create two CTkFrames -----

        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.frame_left = customtkinter.CTkFrame(master=self, width=150, corner_radius=0)
        self.frame_left.grid(row=0, column=0, padx=0, pady=0, sticky="nsew")

        self.frame_right = customtkinter.CTkFrame(master=self, corner_radius=0, fg_color=self.fg_color)
        self.frame_right.grid(row=0, column=1, rowspan=1, pady=0, padx=0, sticky="nsew")

        # ----- Left Frame -----
        self.pakistan_flag_button = customtkinter.CTkButton(master=self.frame_left,
                                                            text="",
                                                            image=self.pakistan_flag,
                                                            fg_color=self.fg_color,
                                                            hover=False)
        self.pakistan_flag_button.grid(pady=(20, 0), padx=(20, 20), row=0, column=0)

        self.naw_logo_button = customtkinter.CTkButton(master=self.frame_left,
                                                       text="",
                                                       image=self.naw_logo,
                                                       fg_color=self.fg_color,
                                                       hover=False)
        self.naw_logo_button.grid(pady=(20, 0), padx=(20, 20), row=1, column=0)

        self.select_area_button = customtkinter.CTkButton(master=self.frame_left,
                                                          text="Select",
                                                          command=self.select_area,
                                                          width=120, height=30,
                                                          border_width=0,
                                                          corner_radius=8,
                                                          fg_color="#0047AB",
                                                          hover_color="grey",
                                                          text_font=App.BUTTON_TEXT_FONT)
        self.select_area_button.grid(pady=(20, 0), padx=(20, 20), row=2, column=0)

        self.generate_plan_button = customtkinter.CTkButton(master=self.frame_left,
                                                            text="Generate",
                                                            command=self.generate_plan,
                                                            width=120, height=30,
                                                            border_width=0,
                                                            corner_radius=8,
                                                            fg_color="#F9AA33",
                                                            hover_color="grey",
                                                            text_font=App.BUTTON_TEXT_FONT)
        self.generate_plan_button.grid(pady=(20, 0), padx=(20, 20), row=3, column=0)

        self.save_plan_button = customtkinter.CTkButton(master=self.frame_left,
                                                        text="Save",
                                                        command=self.save_plan,
                                                        width=120, height=30,
                                                        border_width=0,
                                                        corner_radius=8,
                                                        fg_color="#1DB954",
                                                        hover_color="grey",
                                                        text_font=App.BUTTON_TEXT_FONT)
        self.save_plan_button.grid(pady=(20, 0), padx=(20, 20), row=4, column=0)

        self.clear_plan_button = customtkinter.CTkButton(master=self.frame_left,
                                                         text="Clear",
                                                         command=self.clear_plan,
                                                         width=120, height=30,
                                                         border_width=0,
                                                         corner_radius=8,
                                                         fg_color="#BB000E",
                                                         hover_color="grey",
                                                         text_font=App.BUTTON_TEXT_FONT)
        self.clear_plan_button.grid(pady=(20, 0), padx=(20, 20), row=5, column=0)

        # ----- Right Frame -----

        self.frame_right.grid_rowconfigure(0, weight=1)
        self.frame_right.grid_rowconfigure(1, weight=0)
        self.frame_right.grid_columnconfigure(0, weight=1)
        self.frame_right.grid_columnconfigure(1, weight=0)
        self.frame_right.grid_columnconfigure(2, weight=1)

        self.map_widget = tkintermapview.TkinterMapView(self.frame_right, corner_radius=11)
        self.map_widget.grid(row=0, rowspan=1, column=0, columnspan=3, sticky="nswe", padx=(20, 20), pady=(20, 0))
        self.map_widget.set_address(address_string=App.HOME_COORDS, marker=True, text=App.HOME_NAME)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga",
                                        max_zoom=22)  # Google maps (satellite view)
        self.loader = tkintermapview.OfflineLoader(path=App.MAP_DATABASE_PATH)
        self.loader.save_offline_tiles(App.TOP_LEFT, App.BOTTON_RIGHT, 0, 15)

        self.map_widget.add_right_click_menu_command(label="Add Marker",
                                                     command=self.add_marker_event,
                                                     pass_coords=True)

        self.entry = customtkinter.CTkEntry(master=self.frame_right,
                                            placeholder_text="Type address",
                                            width=140,
                                            height=30,
                                            corner_radius=8)
        self.entry.grid(row=1, column=0, sticky="we", padx=(20, 0), pady=20)
        self.entry.entry.bind("<Return>", self.search_event)

        self.search_button = customtkinter.CTkButton(master=self.frame_right,
                                                     height=30,
                                                     text="Search",
                                                     command=self.search_event,
                                                     border_width=0,
                                                     corner_radius=8,
                                                     hover_color="grey",
                                                     text_font=App.BUTTON_TEXT_FONT)
        self.search_button.grid(row=1, column=1, sticky="w", padx=(20, 0), pady=20)

        self.slider = customtkinter.CTkSlider(master=self.frame_right,
                                              width=400,
                                              height=16,
                                              from_=0, to=19,
                                              border_width=5,
                                              command=self.slider_event,
                                              button_hover_color="grey")
        self.slider.grid(row=1, column=2, sticky="e", padx=20, pady=20)
        self.slider.set(self.map_widget.zoom)

    def search_event(self, event=None):
        """
        This function will be called when search button is pressed.
        """
        # Set the map address to what is specified in search box
        self.map_widget.set_address(self.entry.get())
        # Set the slider value as map zoom value
        self.slider.set(self.map_widget.zoom)

    def slider_event(self, value):
        """
        This function will be called when slider is displaced from its position.
        """
        # Set the slider value as map zoom value
        self.map_widget.set_zoom(value)

    def add_marker_event(self, coords):
        """
        This function will be called when slider is displaced from its position.
        """
        # Set the marker on canvas and append the marker object to marker_list
        self.marker_list.append(self.map_widget.set_marker(coords[0], coords[1],
                                                           text=str((round(coords[0], 4), round(coords[1], 4)))))

    def select_area(self):
        """
        This function will be called when "Select" button is pressed.
        """

        def rotational_sort(coords):
            """
            This function will sort the coordinates into clockwise direction
            """
            coords = np.array(coords)
            cx, cy = coords.mean(0)
            x, y = coords.T
            angles = np.arctan2(x - cx, y - cy)
            indices = np.argsort(angles)
            return coords[indices].tolist()

        # Get coordinates of all markers in marker_list
        marker_coords = []
        for marker in self.marker_list:
            marker_coords.append(marker.position)
        if len(marker_coords) >= 3:
            # Draw the search area as a polygon (non-intersecting) on canvas
            self.search_area_coords = rotational_sort(marker_coords)
            self.map_widget.set_polygon(position_list=self.search_area_coords)
        else:
            print("Please add at least three markers on the map!")

    def generate_plan(self):
        """
        This function will be called when "Generate" button is pressed.
        """

        def get_squares_from_rect(rectangular_polygon, side_length=0.0025):
            """
            Divide a Rectangle (Shapely Polygon) into squares of equal area.

            Args:

            - `side_length` : required side of square

            """
            rect_coords = np.array(rectangular_polygon.boundary.coords.xy)
            y_list = rect_coords[1]
            x_list = rect_coords[0]
            y1 = min(y_list)
            y2 = max(y_list)
            x1 = min(x_list)
            x2 = max(x_list)
            width = x2 - x1
            height = y2 - y1

            xcells = int(np.round(width / side_length))
            ycells = int(np.round(height / side_length))

            yindices = np.linspace(y1, y2, ycells + 1)
            xindices = np.linspace(x1, x2, xcells + 1)
            horizontal_splitters = [
                LineString([(x, yindices[0]), (x, yindices[-1])]) for x in xindices
            ]
            vertical_splitters = [
                LineString([(xindices[0], y), (xindices[-1], y)]) for y in yindices
            ]
            result = rectangular_polygon
            for splitter in vertical_splitters:
                result = MultiPolygon(split(result, splitter))
            for splitter in horizontal_splitters:
                result = MultiPolygon(split(result, splitter))
            square_polygons = list(result)

            return square_polygons

        def split_polygon(G, side_length=0.025, thresh=0.9):
            """
            Using a rectangular envelope around `G`, creates a mesh of squares of required length.

            Removes non-intersecting polygons.

            Args:

            - `thresh` : Range - [0,1]

                This controls - the number of smaller polygons at the boundaries.

                A thresh == 1 will only create (or retain) smaller polygons that are
                completely enclosed (area of intersection=area of smaller polygon)
                by the original Geometry - `G`.

                A thresh == 0 will create (or retain) smaller polygons that
                have a non-zero intersection (area of intersection>0) with the
                original geometry - `G`

            - `side_length` : Range - (0, infinity)
                side_length must be such that the resultant geometries are smaller
                than the original geometry - `G`, for a useful result.

                side_length should be >0 (non-zero positive)

            """
            assert side_length > 0, "side length must be a float>0"
            Rectangle = G.envelope
            squares = get_squares_from_rect(Rectangle, side_length=side_length)
            SquareGeoDF = geopandas.GeoDataFrame(squares).rename(columns={0: "geometry"})
            Geoms = SquareGeoDF[SquareGeoDF.intersects(G)].geometry.values
            geoms = [g for g in Geoms if ((g.intersection(G)).area / g.area) >= thresh]
            return geoms

        def generate_plan_event():
            """
            This function will generate a search plan using midpoints of squares and divide it for n drones
            """
            if int(no_of_drones.get()) > 0 and float(threshold.get()) > 0 and float(threshold.get()) > 0:
                # Define the search area as a shapely polygon
                search_area = Polygon(self.search_area_coords)
                # Split the polygon into squares
                squares = split_polygon(search_area, side_length=float(side_length.get()),
                                        thresh=float(threshold.get()))
                # A variable to store midpoint of squares
                mid_pts = []
                for square in squares:
                    x, y = square.exterior.xy
                    midx = ((x[0] + x[2]) / 2)
                    midy = ((y[0] + y[2]) / 2)
                    mid_pts.append([midx, midy])
                search_paths = np.array_split(mid_pts, int(no_of_drones.get()))
                # Include only the vertices
                search_plan = []
                for search_path in search_paths:
                    vertices = []
                    no_of_points = len(search_path)
                    vertices.append(search_path[0])
                    for point in range(no_of_points):
                        if point is not no_of_points - 1:
                            if search_path[point][1] == search_path[point + 1][1]:
                                continue
                            else:
                                vertices.append(search_path[point])
                                vertices.append(search_path[point + 1])

                    vertices.append(search_path[-1])
                    search_plan.append(vertices)

                    self.search_plan = []
                    for plan in search_plan:
                        plan = [coord.tolist() for coord in plan]
                        self.search_plan.append(plan)

                    # Delete all previous paths from canvas
                    for path in self.path_list:
                        path.delete()

                    # Draw the search path on canvas
                    for i in range(len(search_plan)):
                        if i % 2 == 0:
                            self.path_list.append(
                                self.map_widget.set_path(position_list=search_plan[i], color="#1DB954"))
                        else:
                            self.path_list.append(
                                self.map_widget.set_path(position_list=search_plan[i], color="#BB000E"))
            else:
                print("Configuration not valid")

        if self.search_area_coords:
            # Define the configuration window
            configuration_window = customtkinter.CTkToplevel()
            configuration_window.title("Search Configuration")
            configuration_window_size = (400, 400)
            configuration_window.geometry(str(configuration_window_size[0]) + "x" + str(configuration_window_size[1]))
            configuration_window.minsize(configuration_window_size[0], configuration_window_size[1])

            no_of_drones = customtkinter.CTkEntry(master=configuration_window,
                                                  placeholder_text="Number of drones (ex. 2)",
                                                  corner_radius=8)
            no_of_drones.pack(side="top", fill="both", expand=True, padx=20, pady=20)

            side_length = customtkinter.CTkEntry(master=configuration_window,
                                                 placeholder_text="Side length of the squares (ex. 0.0001)",
                                                 corner_radius=8)
            side_length.pack(side="top", fill="both", expand=True, padx=20, pady=20)

            threshold = customtkinter.CTkEntry(master=configuration_window,
                                               placeholder_text="Threshold (ex. 0.5)",
                                               corner_radius=8)
            threshold.pack(side="top", fill="both", expand=True, padx=20, pady=20)

            accept_button = customtkinter.CTkButton(master=configuration_window,
                                                    height=30,
                                                    text="Accept",
                                                    command=generate_plan_event,
                                                    border_width=0,
                                                    corner_radius=8,
                                                    fg_color="#1DB954",
                                                    hover_color="grey",
                                                    text_font=App.BUTTON_TEXT_FONT)
            accept_button.pack(side="top", fill="both", expand=True, padx=20, pady=10)

            close_button = customtkinter.CTkButton(master=configuration_window,
                                                   height=30,
                                                   text="Close",
                                                   command=configuration_window.destroy,
                                                   border_width=0,
                                                   corner_radius=8,
                                                   fg_color="#BB000E",
                                                   hover_color="grey",
                                                   text_font=App.BUTTON_TEXT_FONT)
            close_button.pack(side="top", fill="both", expand=True, padx=20, pady=10)

            configuration_window.wm_iconphoto(False, self.naw_logo)
        else:
            print("Please select the search area first!")

    def clear_plan(self):
        """
        This function will be called when "Clear" button is pressed.
        """
        # Delete all markers from canvas
        for marker in self.marker_list:
            marker.delete()
        # Clear the marker list
        self.marker_list = []
        # Clear the search area coordinates
        self.search_area_coords = []
        # Delete all polygons from canvas
        for polygon in self.map_widget.canvas_polygon_list:
            polygon.delete()
        # Delete all paths from canvas
        for path in self.path_list:
            path.delete()

    def save_plan(self):
        """
        This function will be called when "Save" button is pressed.
        """

        for plan_no, plan in enumerate(self.search_plan):
            print("Plan No. " + str(plan_no) + ":", plan)

        with open(App.PLAN_SAVE_PATH, "r", encoding="utf-8") as search_plan_file:
            plan_dict = json.load(search_plan_file)

        plan_dict["search_plan"] = self.search_plan

        with open(App.PLAN_SAVE_PATH, "w+", encoding="utf-8") as sar_plan_file:
            sar_plan_file.write(json.dumps(plan_dict, indent=4))

        print("Search plan saved successfully!")

        if App.GENERATE_SURVEY_MISSION:
            sitl = SITL()
            sitl.download('copter', '3.3', verbose=True)
            sitl_args = ['-I0', '--model', 'quad', '--home=' + App.HOME_COORDS + ',0,180']
            sitl.launch(sitl_args, await_ready=True, restart=True)

            connection_string = 'tcp:127.0.0.1:5760'
            print("Connecting to vehicle on: %s" % connection_string)
            vehicle = connect(connection_string, wait_ready=True, baud=57600)

            # Check that vehicle is armable. This ensures home_location is set.
            while not vehicle.is_armable:
                print("Waiting for vehicle to initialise...")
                time.sleep(1)

            cmds = vehicle.commands

            cmds.clear()
            home = vehicle.location.global_frame
            home = LocationGlobal(lat=float(App.HOME_COORDS.split(",")[0]), lon=float(App.HOME_COORDS.split(",")[1]), alt=0)
            vehicle.home_location = home
            mission_altitude = 5  # m

            # Take off  command
            cmds.add(
                Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                        0, 0,
                        0, 0, 0, 0, mission_altitude))

            survey = []
            for plan in self.search_plan:
                for coord in plan:
                    survey.append(coord)

            for coord in survey:
                cmds.add(
                    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                            0, 0, 0, 0, 0, coord[0], coord[1], mission_altitude))

            # Land command
            cmds.add(
                Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                        0, 0, 0,
                        0, home.lat, home.lon, 0))

            # Save the mission in waypoint file format
            print("Saving mission file to ", App.MISSION_SAVE_PATH, " ...")
            output = 'QGC WPL 110\n'
            for cmd in cmds:
                commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
                    cmd.seq, cmd.current, cmd.frame, cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.x,
                    cmd.y,
                    cmd.z, cmd.autocontinue)
                output += commandline
            with open(App.MISSION_SAVE_PATH, 'w') as file:
                file.write(output)

            vehicle.close()
            print("Done!")

    def on_closing(self, event=0):
        # Release the App object from memory
        self.destroy()

    def start(self):
        # Set NAW logo as app icon
        self.wm_iconphoto(False, self.naw_logo)
        # Loop forever
        self.mainloop()


def main():
    """
    This function contains the main code.
    """
    # Create an object of the App class
    app = App()
    # Start the app
    app.start()


if __name__ == "__main__":
    main()
