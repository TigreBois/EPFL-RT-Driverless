# Simulation
## Available now:
* Webots simulation of simple track for sensor data recording.
* Generattion of real track world file for Webots simulator
## Files description
* ```track generation.ipynb``` - Notebook is used for generation of tracks. It's possible to generate two types of tracks. First type is generated from csv file with trajectory points. Second type is circle track with simple vehicle made for recording of sensors data.
* ```generated_track.wbt``` - Webots world file which contains real track
* ```generated_circle_track.wbt``` - Webots world file which contains circle track with simple vehicle for recording of sensors data

* ```circle_track_controller.py``` - Controler file for simulation of circle track
* ```circle_track_cones_coordinates.csv``` - CSV file which contains info about positions of generated cones for circle track simulation. Used for precise localization of cones while recording sensor data.
* ```circle_track_sensors_data.pickle``` - Recorded sensor data
