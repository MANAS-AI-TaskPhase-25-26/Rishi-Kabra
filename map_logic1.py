import numpy as np
import cv2

print("Loading smaller_map.csv...")
data = np.loadtxt("elevation_map.csv", delimiter=",", skiprows=1)

raw_x = data[:, 0]
raw_y = data[:, 1]
elevations = data[:, 2]

x_meters = raw_x - np.min(raw_x)
y_meters = raw_y - np.min(raw_y)

scale = 10      
padding = 50        

max_x = np.max(x_meters)
max_y = np.max(y_meters)

img_height = int(max_x * scale) + (padding * 2)
img_width = int(max_y * scale) + (padding * 2)

print(f"Calculated Field Size: {max_y:.1f}m East-West x {max_x:.1f}m North-South")
print(f"Generating Image Size: {img_width}x{img_height} pixels")

canvas = np.zeros((img_height, img_width), dtype=np.uint8)

z_min = np.percentile(elevations, 0)
z_max = np.percentile(elevations, 100)
print(f"Filtered Elevation Range: {z_min:.2f}m to {z_max:.2f}m")

for i in range(len(data)):
    m_north = x_meters[i]
    m_east = y_meters[i]
    z = elevations[i]
    
    col = int(m_east * scale) + padding
    row = img_height - (int(m_north * scale) + padding)
    
    if 0 <= col < img_width and 0 <= row < img_height:
        if z_max > z_min:
            z_clipped = max(z_min, min(z_max, z))
            brightness = int(((z_clipped - z_min) / (z_max - z_min)) * 254) + 1
        else:
            brightness = 127 
            
        canvas[row, col] = brightness

kernel_size = 3
dilated = cv2.dilate(canvas, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)
final_map = cv2.applyColorMap(dilated, cv2.COLORMAP_JET)
final_map[dilated == 0] = [0, 0, 0]
output_filename = "Depth_Map_1.png"
cv2.imwrite(output_filename, final_map)
print(f"SUCCESS: Map saved as '{output_filename}'")