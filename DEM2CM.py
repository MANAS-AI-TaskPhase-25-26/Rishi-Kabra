import cv2
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

def pipeline(img_path):
    jet_img = cv2.imread(img_path)
    if jet_img is None:
        print("Image not found.")
        return
    
    indices_ref = np.arange(256).astype(np.uint8).reshape(1, 256)
    jet_palette = cv2.applyColorMap(indices_ref, cv2.COLORMAP_JET).squeeze()
    tree = KDTree(jet_palette)

    h, w, _ = jet_img.shape
    pixels = jet_img.reshape(-1, 3)
    _, raw_indices = tree.query(pixels)
    elevation_raw = raw_indices.reshape(h, w)
    standard_numerical_grid = (255 - elevation_raw).astype(np.uint8)
    standard_vis_rgb = cv2.applyColorMap(standard_numerical_grid, cv2.COLORMAP_MAGMA)

    return jet_img, standard_numerical_grid, standard_vis_rgb

path = 'Depth_Map_1.png'
original_jet, numeric_grid, standard_vis = pipeline(path)

plt.figure(figsize=(15, 5))

plt.subplot(1, 3, 1)
plt.title("Jet Map")
plt.imshow(cv2.cvtColor(original_jet, cv2.COLOR_BGR2RGB))
plt.axis('off')

plt.subplot(1, 3, 2)
plt.title("Numerical Grid")
plt.imshow(numeric_grid, cmap='gray')
plt.axis('off')

plt.subplot(1, 3, 3)
plt.title("Cost Map")
plt.imshow(cv2.cvtColor(standard_vis, cv2.COLOR_BGR2RGB))
plt.axis('off')

plt.tight_layout()
plt.show()

cv2.imwrite('costmap_visual.png', standard_vis)
print("Saved costmap")

np.save('aStar_data.npy', numeric_grid)
print("Saved map_for_astar.npy.")