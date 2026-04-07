import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Add the directory of the current script to the system path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

def fractal_dimension(Z, threshold=0.9):
    # Only for 2D binary images
    assert(len(Z.shape) == 2)

    # Convert to binary: foreground (structure) should be 1
    Z = (Z < threshold * 255)
        
    # Visualize Z
    Z_vis = Z.astype(np.uint8) * 255
    plt.figure()
    plt.imshow(Z_vis, cmap='gray')
    plt.title('Processed Binary Structure (Z)')
    plt.axis('off')
    plt.show()

    def boxcount(Z, k):
        S = np.add.reduceat(
            np.add.reduceat(Z, np.arange(0, Z.shape[0], k), axis=0),
                               np.arange(0, Z.shape[1], k), axis=1)

        # Count non-empty (at least one foreground pixel) and non-full boxes
        return len(np.where((S > 0) & (S < k*k))[0])

    # Minimal dimension of image
    p = min(Z.shape)
    # Greatest power of 2 less than or equal to p
    n = 2**np.floor(np.log2(p))
    n = int(n)
    Z = Z[:n, :n]

    sizes = 2**np.arange(int(np.log2(n)), 1, -1)
    counts = [boxcount(Z, size) for size in sizes]

    # Fit line to log-log data
    coeffs = np.polyfit(np.log(sizes), np.log(counts), 1)
    fractal_dim = -coeffs[0]
    
    # Optional: show plot
    plt.figure()
    plt.plot(np.log(sizes), np.log(counts), 'o-', label=f'D ≈ {fractal_dim:.4f}')
    plt.xlabel('log(Box size)')
    plt.ylabel('log(Count)')
    plt.legend()
    plt.show()

    return fractal_dim

# Load image
image_path = "outputs\structure.png"
gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Thresholding (convert to binary), return threshold val it used, and thresholded image
_, binary = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)


# Compute fractal dimension
D = fractal_dimension(binary)
print(f"Fractal Dimension: {D:.4f}")
