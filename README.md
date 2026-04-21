## Execution pipeline
### Connecting
* open Africa_001 world in a terminal
* in another terminal connect to the airsim using ardupilot with this comnand:
```bash
sim_vehicle.py -v ArduCopter -f airsim-copter --console --map
```
* then open a new terminal and and run these scripts in order with the 'boundary.txt' file in the same folder as the scripts (also the lat and lon are only taken out correctly from the txt if its in the same format as the one given because thats how i wrote the code to parse the strings i.e. 
Point 1 = Lat:-35.3631093 Lon:149.1648396
* i changed the param settings in ardupilot and stored the setting as 'projectChanges.param'. so after opening up ardupilot sitl load up the changes using this command:
```bash
 param load my_project_config.param
```
* after that just do 'arm throttle' in ardu pilot terminal because arming doesn't work for some reason in my code. after its armed throught the terminal, everything else works perfectly fine throught code. the script execution sequence is as follows:

#### 1. DEM.py 
pulls the lat and lon coordinates of the boundary specified from a 'boundary.txt' file. Connects to airsim and uses dronekit to give maneuver commands. uses the shapely library to build a polygon using the coordinates and appends all the points inside the polygon into an array (polygon.contains() using ray casting ). using this points we also build a visual boundary in the airsim and mark the approximate points the drone is going to stop and stabalize to take a particular lidar scan. the drone is initially taken to a altitude of 9m and flown over the boundary area using lawnmover path. while flying the drone gets lidar data at equal intervals. all the lidar points are then saved to a csv file ( 'elevation_map.csv' ) with thier corresponding x and y value in the airsim world so that they can be mapped into a image later. 
### 2. map_logic1.py
takes the data from the 'elevation_file.csv' and stores into a numpy array for faster calculations. the minimum and the maximum values are taken from the file. minumum is set as 0,0 in the image and max is used to find the img height and width. all the points are then mapped propotionally on the image relative to the min and max points. padding is used to give some space between map and image window boundary. lidar points pixels are scaled up so that they are more clearly visible. kernel is also used to dilate the pixels so that the empty pixels can be filled. final image is stored as 'Depth_Map_1.png' with cv2's JET palette

### 3. DEM2CM.py 
The jetmap image is coverted using this script into greyscale and a costmap. scipy KDTree is used to covert jetmap image back to numbers since jetmap ranges from green to yellow to blue to red and is not a linear scale like grayscale. it stores the elevation height data as 'aStar_data.npy' and the costmap as 'costmap_visual.png'. it also displays all the 3 maps using matplotlib.

### 4. xyfinder.py 
lets you choose 2 points (start and goal) on the visual map and stores the point clicked coordinates in 'pathPoints.txt' which is then picked up by the aStar script when running it and it finds the best path between those points (exactly 2 points should be clicked tho)
### 5. aStar.py 
Uses the points extracted from 'pathPoints.txt' to map the path. uses aStar ( distance penalty and elevation penalty ) . uses heapq data structure ( works using a tree ) to store and sort the queue for a faster algorithm. heappush and heappop both having O(logn) the displays the path using matplotlib.

## Libraries used
* numpy
* opencv-python
* scipy
* matplotlib
* shapely
* airsim
* dronekit
```bash 
pip install numpy opencv-python scipy matplotlib shapely airsim dronekit
```

