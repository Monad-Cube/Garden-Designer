from base import *
from matplotlib.patches import Polygon as Poly
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
from matplotlib.widgets import Button
from matplotlib.gridspec import GridSpec
from shapely.geometry import Polygon, Point
import copy
import random

# def xy2idx(x: float, y: float) -> Tuple[float, float]:
#     """
#     transfer the point position (*RL) to MAP_WH coordinate
#     height_map(0,0) is at top left corner
#     """
#     # Map from [-RL, (W+2)*RL] to [0, MAP_W-1]
#     # Map from [-RL, (H+2)*RL] to [0, MAP_H-1]
#     # Scale to use the full height map size
#     x_idx = (x + RL) * (MAP_W - 1) / ((W + 2) * RL)
#     y_idx = (y + RL) * (MAP_H - 1) / ((H + 2) * RL)
    
#     # Ensure we're using the full height map
#     x_idx = x_idx * MAP_W / (W + 2)
#     y_idx = y_idx * MAP_H / (H + 2)
    
#     return x_idx, y_idx

def xy2idx(x: float, y: float) -> Tuple[float, float]:
    """
    transfer the point position (*RL) to MAP_WH coordinate
    height_map is initialized as [[0 for _ in range(MAP_H)] for _ in range(MAP_W)]
    """
    # Map from [-RL, (W+2)*RL] to [0, MAP_W]
    # Map from [-RL, (H+2)*RL] to [0, MAP_H]
    x_idx = (x + RL) * MAP_W / ((W + 2) * RL)
    y_idx = (y + RL) * MAP_H / ((H + 2) * RL)
    
    # Ensure we stay within bounds
    x_idx = min(max(x_idx, 0), MAP_W - 1)
    y_idx = min(max(y_idx, 0), MAP_H - 1)
    
    return x_idx, y_idx


def override_height(
    height_map: List[List[float]], poly: Polygon, height: float, buffer: float
) -> List[Tuple[int, int]]:
    """
    modify values in height_map
    region in poly will be set to height
    units near poly will be smoothed, buffer is the distance
    """
    points = [xy2idx(x, y) for x, y in poly.exterior.coords]
    idx_poly = Polygon(points)
    # decide what units are covered by the idx_poly
    minx, miny, maxx, maxy = idx_poly.bounds
    ratio = max(MAP_W, MAP_H) / (RL * max(W, H))
    x1, z1 = int(minx - buffer * ratio), int(miny - buffer * ratio)
    x2, z2 = int(maxx + buffer * ratio) + 1, int(maxy + buffer * ratio) + 1
    idxs = []
    for i in range(x1, x2):
        for j in range(z1, z2):
            if i < 0 or i >= MAP_W or j < 0 or j >= MAP_H:
                continue
            pcenter = Point(i + 0.5, j + 0.5)
            if idx_poly.contains(pcenter):
                height_map[i][j] = height
                idxs.append((i, j))
            else:
                dis = pcenter.distance(idx_poly) / ratio
                if dis < buffer:
                    height_map[i][j] = height * (1 - dis / buffer) + height_map[i][j] * dis / buffer

    return idxs

def update_height_map(height_map, points_array, areas):
    """Update height map based on modified points and areas"""
    # For each area that was modified
    for area_type, circuit in areas:
        # Get the points for this area
        area_points = points_array[circuit]
        
        # Determine target height based on area type
        target_height = 0.1  # Default height for land
        # if area_type == 0: 
        #     target_height = WATER_HEIGHT
        # elif area_type == 1:  # Water
        #     target_height = 0.0
        # elif area_type == 2:  # land
        #     target_height = 0.1
        # elif area_type == 3:  # ground
        #     target_height = 0.1
        target_height = 1
        # Create polygon directly from the points
        poly = Polygon(area_points)
        
        # Update height map in the affected region
        override_height(height_map, poly, target_height, buffer=0)
    
    return height_map

def edit(
    height_map: List[List[float]],
    points: List[Tuple[float, float]],
    edges: List[Tuple[List[Tuple[int, int]], List[int]]],
    areas: List[Tuple[int, List[int]]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[int, int]], dict],
    colors: List[Tuple[float,float]], # (r,g,b) range(0,1)
    data: dict,
    gen_idx: int,
    suffix: str = "",
) -> Tuple[List[List[float]], List[Tuple[float, float]], List[Tuple[List[Tuple[int, int]], List[int]]], List[Tuple[int, List[int]]]]:
    # Create figure and make it interactive
    plt.ion()
    fig = plt.figure(1) # Access the auto-generated blank figure
    fig.set_size_inches(19.2, 10.8)  # Width = 1920px / 100 dpi, Height = 1080px / 100 dpi
    fig.set_dpi(100)  # Set DPI to 100 (default in Matplotlib)
    axs = plt.gca()                      # Get current axes
    axs.set_aspect('equal')              # Set aspect ratio
    axs.axis('off')                      # Not display axis

    # Create GridSpec with different column widths (width_ratios)
    gs = GridSpec(1, 3, width_ratios=[1, 3, 0.5])  # 1 row, 3 columns with different widths

    # Make deep copies of the input data
    new_height_map = copy.deepcopy(height_map)
    new_points = copy.deepcopy(points)
    new_edges = copy.deepcopy(edges)
    new_areas = copy.deepcopy(areas)

    # Create subplots from the GridSpec
    ax = [fig.add_subplot(gs[i]) for i in range(3)]
    for i in range(3): 
        ax[i].axis('off') # Not display axis               

    #------------------- Function Area ------------------#
    ax[0].set_title("Function")
    # Create button and set position
    button_ax = plt.axes([0.05, 0.8, 0.075, 0.05])  # Adjusted for first column
    
    # Define a function to execute when button is clicked
    def on_button_clicked(event):
        print("Button clicked!")

    button = Button(button_ax, 'Test Button')
    button.on_clicked(on_button_clicked)


    #--------------------- Drawing --------------------#
    ax[1].set_title("Editing")
    # Convert to numpy array for easier manipulation
    points_array = np.array(new_points)     # [(x,y)
    flattened_edges = [p for edge_list, _ in new_edges for p in edge_list] # 
    area_types = np.array([area[0] for area in new_areas])  
    area_circuit = [np.array(area[1]) for area in new_areas]  
    
    selected_point = None
    selected_area = None
    modified_areas = set()  # Track which areas have been modified
    
    # color
    point_default_color = "#52966b"
    point_selected_color = "red"
    edge_default_color =  "#468f85"
    area_selected_color = "red"

    # Plot elements
    scatter = ax[1].scatter(points_array[:, 0], points_array[:, 1], c=point_default_color, s=20, zorder=3)

    # Plot edges efficiently, embed two for loop in []
    edge_lines = [
        ax[1].plot(
            points_array[[p1_idx, p2_idx], 0],
            points_array[[p1_idx, p2_idx], 1],
            linewidth=1, zorder=1, color=edge_default_color
        )[0]
        for edge_list, _ in new_edges
        for p1_idx, p2_idx in edge_list
    ]

    # Efficient area plotting, similarly
    area_patches = [
        ax[1].add_patch(Poly(
            points_array[circuit],
            color=colors[area_type],
            alpha=0.3,
            zorder=0
        ))
        for area_type, circuit in new_areas
        if len(circuit) >= 3
    ]

    def update_visualization():
        scatter.set_offsets(points_array) # update point location

        for line, (p1_idx, p2_idx) in zip(edge_lines, flattened_edges):
            edge_points = np.array([points_array[p1_idx], points_array[p2_idx]])
            line.set_data(edge_points[:, 0], edge_points[:, 1])  # Just update data

        for patch, (_, circuit) in zip(area_patches, new_areas):
            patch.set_xy(points_array[circuit])

        fig.canvas.draw_idle()

    def on_click(event):
        nonlocal selected_point, selected_area
        if event.inaxes != ax[1]:
            return

        dist = np.linalg.norm(points_array - [event.xdata, event.ydata], axis=1)
        nearest_idx = np.argmin(dist)
        # select points
        if dist[nearest_idx] < 0.1:
            selected_point = nearest_idx
            scatter.set_facecolors([point_selected_color if i == nearest_idx else point_default_color 
                                    for i in range(len(new_points))])    
            # Find which areas contain this point
            for i, circuit in enumerate(area_circuit):
                if nearest_idx in circuit:
                    modified_areas.add(i)
        # select area
        else:
            for i, circuit in enumerate(area_circuit):
                polygon = points_array[circuit]  # Get area coordinates
                path = Poly(polygon).get_path()

                if path.contains_point([event.xdata, event.ydata]):
                    selected_area = i
                    area_patches[i].set_facecolor(area_selected_color)
                    modified_areas.add(i)
                    break
        fig.canvas.draw_idle()
             
    def on_release(_):
        nonlocal selected_point, selected_area, new_height_map, points_array
        if selected_point is not None:
            scatter.set_facecolors(point_default_color)
            # Update height map for all modified areas
            for area_idx in modified_areas:
                area_type, circuit = new_areas[area_idx]
                area_points = points_array[circuit]
                poly = Polygon(area_points)
                
                # Determine target height based on area type
                target_height = 0.1  # Default height for land
                if area_type == 0: 
                    target_height = WATER_HEIGHT
                elif area_type == 1:  # Water
                    target_height = 0.0
                elif area_type == 2:  # land
                    target_height = 0.1
                elif area_type == 3:  # ground
                    target_height = 0.1
                
                # Update height map for this area
                override_height(new_height_map, poly, target_height, buffer=0)
            
            selected_point = None
            modified_areas.clear()
            fig.canvas.draw_idle()

        if selected_area is not None:
            area_patches[selected_area].set_facecolor(colors[area_types[selected_area]])
            selected_area = None
            fig.canvas.draw_idle()

    def on_motion(event):
        nonlocal points_array
        if selected_point is not None and event.inaxes == ax[1]:
            points_array[selected_point] = [event.xdata, event.ydata]
            update_visualization()

    def on_close(_):
        plt.close('all')

    # Event connections
    fig.canvas.manager.set_window_title('Edit')
    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('close_event', on_close)


    # Set plot limits with padding
    padding = 2
    ax[1].set_xlim(points_array[:, 0].min() - padding, points_array[:, 0].max() + padding)
    ax[1].set_ylim(points_array[:, 1].min() - padding, points_array[:, 1].max() + padding)

    #------------------- Explanation ---------------------#
    ax[2].set_title("Explanation")

    plt.show(block=True)

    # Convert points_array back to list of tuples
    new_points = [tuple(p) for p in points_array]

    return new_height_map, new_points, new_edges, new_areas
