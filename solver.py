from base import *
import copy
import datetime
import json
import math
import random
import time

import matplotlib.pyplot as plt
import numpy as np
from rtree import index
from scipy.interpolate import interp1d
from shapely.geometry import Polygon, Point, box, LineString

class DFS_Solver:
    def __init__(self, grid_size, height_map, random_seed=0, max_duration=5, constraint_bouns=0.2):
        self.grid_size = grid_size
        self.random_seed = random_seed
        self.max_duration = max_duration  # maximum allowed time in seconds
        self.constraint_bouns = constraint_bouns
        self.start_time = None
        self.solutions = []
        self.vistualize = False
        self.height_map = height_map
        # Define the functions in a dictionary to avoid if-else conditions
        self.func_dict = {
            "global": self.place_global,
            "position": self.place_relative,
            "rotation": self.place_face,
            "alignment": self.place_alignment_center,
            "distance": self.place_distance,
        }

        self.constraint_type2weight = {
            "global": 2.0,
            "position": 0.5,
            "rotation": 0.5,
            "alignment": 0.5,
            "distance": 1.8,
        }

        self.edge_bouns = 0.0  # worth more than one constraint

    def get_solution(
        self, area, objects_list, constraints, initial_state = {}
    )-> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
        """
        input: area info, object list for area, object constrains
        output: obj_infos_list for entire area: [([x,y,z,forw], object_real_name),...]
        """
        initial_state = {}

        self.start_time = time.time()
        bounds = area
        grid_points = self.create_grids(bounds) # room polygon
        output_solution = []
        remaining_area = area
        try:
            self.dfs(
                bounds, objects_list, constraints, grid_points, initial_state, 30
            )
        except SolutionFound as e:
            print(f"Time taken: {time.time() - self.start_time}")

        print(f"Number of solutions found: {len(self.solutions)}")
        if len(self.solutions) !=0:
            min_solution = self.get_min_solution(self.solutions)
            output_solution = self.transfer_ouput_solution(min_solution, self.height_map)
            remaining_area, _ = self.calculate_remaining_area(output_solution, objects_list, area)

        if self.vistualize:
            self.visualize_grid(bounds, grid_points, min_solution)

        return output_solution, remaining_area
        
    def transfer_ouput_solution(self, solutions, height_map):
        transfered_solution = []
        for obj_name, obj_info in solutions.items():
            x, z = obj_info[0]
            y = self.get_height(height_map, x, z) * MAX_HEIGHT
            if obj_info[1] == 0: forw = 0
            elif obj_info[1] == 90: forw = -pi/2
            elif obj_info[1] == 180: forw = pi
            elif obj_info[1] == 270: forw = pi/2
            cleaned_obj_name = re.sub(r'-\d+$', '', obj_name)
            transfered_solution.append(([(x,y,z,forw)], cleaned_obj_name))
        return transfered_solution
    
    def calculate_remaining_area(self, output_solution, object_list, original_poly):
        """
        Calculate the remaining area after placing objects
        
        Parameters:
        -----------
        output_solution: List[((center_x, bottom_y, center_z, forward), object_name)]
            List of placed objects with their positions and orientations
        object_list: List[(object_name, (length, width))]
            List of objects with their dimensions
        original_poly: Polygon
            The original polygon representing the available area
        
        Returns:
        --------
        remaining_poly: Polygon
            The largest polygon representing the remaining area
        used_area: float
            The area used by placed objects
        """
        object_dict = {}
        # Convert object_list to a dictionary for easy lookup
        for name, (length, width) in object_list:
            cleaned_obj_name = re.sub(r'-\d+$', '', name)
            object_dict[cleaned_obj_name] = (length, width)
        
        placed_polygons = []
        
        # Create a polygon for each placed object
        for object_info, object_name in output_solution:
            center_x, _, center_z, forward = object_info[0]
            # Get object dimensions
            length, width = object_dict[object_name]
            
            # Adjust dimensions based on orientation
            if forward in [0,pi]:  # Forward orientation
                l, w = length, width
            else:  # Rotated orientation
                l, w = width, length
            
            # Calculate the corners of the rectangle
            # For a rectangle centered at (center_x, center_z) with length l and width w
            min_x = center_x - l/2
            max_x = center_x + l/2
            min_z = center_z - w/2
            max_z = center_z + w/2
            
            # Create a rectangle for this object
            rect = box(min_x, min_z, max_x, max_z)
            placed_polygons.append(rect)
        
        # Union of all placed objects
        if placed_polygons:
            used_area_poly = unary_union(placed_polygons)
        else:
            used_area_poly = Polygon()
        
        # Calculate the remaining area
        remaining_poly = original_poly.difference(used_area_poly)
        
        # If the result is a multipolygon, get the largest polygon
        if remaining_poly.geom_type == 'MultiPolygon':
            remaining_poly = max(remaining_poly.geoms, key=lambda x: x.area)
        
        return remaining_poly, used_area_poly.area


    def get_min_solution(self, solutions):
        path_weights = [sum(obj[-1] for obj in solution.values()) for solution in solutions]
        min_value = np.min(path_weights)
        min_indices = np.where(path_weights == min_value)[0]
        min_index = np.random.choice(min_indices)
        return solutions[min_index]

    def dfs(
        self,
        area_poly,
        objects_list,
        constraints,
        grid_points,
        placed_objects,
        branch_factor, # how many branch
    ):
        if len(objects_list) == 0: # ultimately, append
            self.solutions.append(placed_objects)
            return placed_objects

        if time.time() - self.start_time > self.max_duration:
            print(f"Time limit reached.")
            raise SolutionFound(self.solutions)
        # handle the first object in objects list, note that
        object_name, object_dim = objects_list[0]
        placements = self.get_possible_placements(
            area_poly, object_dim, constraints[object_name], grid_points, placed_objects
        )

        if len(placements) == 0 and len(placed_objects) != 0: # no space for left objects, with some placed objects 
            self.solutions.append(placed_objects)

        paths = []
        if branch_factor > 1:
            random.shuffle(placements)  # shuffle the placements of the first object

        for placement in placements[:branch_factor]:
            placed_objects_updated = copy.deepcopy(placed_objects)
            placed_objects_updated[object_name] = placement
            grid_points_updated = self.remove_points(
                grid_points, placed_objects_updated
            )

            sub_paths = self.dfs(
                area_poly,
                objects_list[1:], # decrease one object at each iteraction, and at last it will return empty object_list
                constraints,
                grid_points_updated,
                placed_objects_updated,
                1,
            )
            paths.extend(sub_paths)

        return paths

    def get_possible_placements(
        self, area_poly, object_dim, constraints, grid_points, placed_objects
    ):
        # (x,y), rotation[0,90,180,270], (5 box coords), 1(start score)
        solutions = self.get_all_solutions(area_poly, grid_points, object_dim)
        if len(placed_objects) != 0:
            solutions = self.filter_collision(
                placed_objects, solutions
            )
        #solutions = self.filter_facing_wall(area_poly, solutions, object_dim)
        # edge solution could be removed
        #edge_solutions = self.place_edge(area_poly, copy.deepcopy(solutions), object_dim)
        #if len(edge_solutions) == 0:
        #    return edge_solutions
         
        candidate_solutions = copy.deepcopy(solutions)  # the first object

        if candidate_solutions == []:
            return candidate_solutions
        random.shuffle(candidate_solutions)
        placement2loss = {tuple(solution[:3]): solution[-1] for solution in candidate_solutions} # init

        # Specially Processing for Global Constraints
        # HACK, i think there is no collision detection?

        for constraint in constraints:
            func = self.func_dict.get(constraint[-1])
            # NOTE, global optimazation
            if constraint[-1] == "global":
                valid_solutions = func(
                    constraint[0],
                    area_poly,
                    candidate_solutions,
                    object_dim
                )
            else:
                constraint_content, target, constraint_type = constraint[0], constraint[1], constraint[2]
                # TODO, specially processing for water area
                if constraint_type == "rotation" and target == "water area":
                    valid_solutions = func(
                        area_poly,
                        constraint_content,
                        "water_area",
                        candidate_solutions,
                    )
                else:
                    valid_solutions = func(
                        area_poly,
                        constraint_content,
                        placed_objects[target],
                        candidate_solutions,
                    )

            weight = self.constraint_type2weight[constraint[-1]]
            for solution in valid_solutions:
                loss = solution[-1]
                placement2loss[tuple(solution[:3])] += loss * weight
        # sorting from smallest to biggest
        sorted_placements = sorted(
            placement2loss, key=placement2loss.get, reverse=False
        )
        sorted_solutions = [
            list(placement) + [placement2loss[placement]]
            for placement in sorted_placements
        ]

        return sorted_solutions

    def create_grids(self, area_poly):
        # get the min and max bounds of the room
        min_x, min_z, max_x, max_z = area_poly.bounds

        # create grid points
        grid_points = []
        # for x in range(int(min_x), int(max_x), self.grid_size):
        #     for y in range(int(min_z), int(max_z), self.grid_size):
        #         point = Point(x, y)
        #         if area_poly.contains(point):
        #             grid_points.append((x, y))
        for x in np.arange(min_x, max_x, self.grid_size):
            for y in np.arange(min_z, max_z, self.grid_size):
                point = Point(x, y)
                if area_poly.contains(point):
                    grid_points.append((x, y))
        return grid_points

    def remove_points(self, grid_points, objects_dict):
        # Create an r-tree index
        idx = index.Index()

        # Populate the index with bounding boxes of the objects
        for i, (_, _, obj, _) in enumerate(objects_dict.values()):
            idx.insert(i, Polygon(obj).bounds)

        # Create Shapely Polygon objects only once
        polygons = [Polygon(obj) for _, _, obj, _ in objects_dict.values()]

        valid_points = []

        for point in grid_points:
            p = Point(point)
            # Get a list of potential candidates
            candidates = [polygons[i] for i in idx.intersection(p.bounds)]
            # Check if point is in any of the candidate polygons
            if not any(candidate.contains(p) for candidate in candidates):
                valid_points.append(point)

        return valid_points

    def get_all_solutions(self, area_poly, grid_points, object_dim):
        # based on anchor object, we find all the feasible solution
        obj_length, obj_width  = object_dim
        obj_half_length, obj_half_width = obj_length / 2, obj_width / 2

        rotation_adjustments = {
            0: ((-obj_half_length, -obj_half_width), (obj_half_length, obj_half_width)),
            90: (
                (-obj_half_width, -obj_half_length),
                (obj_half_width, obj_half_length),
            ),
            180: (
                (-obj_half_length, obj_half_width),
                (obj_half_length, -obj_half_width),
            ),
            270: (
                (obj_half_width, -obj_half_length),
                (-obj_half_width, obj_half_length),
            ),
        }

        solutions = []
        for rotation in [0, 90, 180, 270]:
            for point in grid_points:
                center_x, center_y = point
                lower_left_adjustment, upper_right_adjustment = rotation_adjustments[
                    rotation
                ]
                lower_left = (
                    center_x + lower_left_adjustment[0],
                    center_y + lower_left_adjustment[1],
                )
                upper_right = (
                    center_x + upper_right_adjustment[0],
                    center_y + upper_right_adjustment[1],
                )
                obj_box = box(*lower_left, *upper_right)

                if area_poly.contains(obj_box):
                    solutions.append(
                        [point, rotation, tuple(obj_box.exterior.coords[:]), 0] 
                    ) # i think the tuple is not so important

        return solutions

    def filter_collision(self, objects_dict, solutions):
        valid_solutions = []
        object_polygons = [
            Polygon(obj_coords) for _, _, obj_coords, _ in list(objects_dict.values())
        ]
        for solution in solutions:
            sol_obj_coords = solution[2]
            sol_obj = Polygon(sol_obj_coords)
            if not any(sol_obj.intersects(obj) for obj in object_polygons):
                valid_solutions.append(solution)
        return valid_solutions

    def filter_facing_wall(self, area_poly, solutions, obj_dim):
        valid_solutions = []
        obj_width = obj_dim[1]
        obj_half_width = obj_width / 2

        front_center_adjustments = {
            0: (0, obj_half_width),
            90: (obj_half_width, 0),
            180: (0, -obj_half_width),
            270: (-obj_half_width, 0),
        }

        valid_solutions = []
        for solution in solutions:
            center_x, center_y = solution[0]
            rotation = solution[1]

            front_center_adjustment = front_center_adjustments[rotation]
            front_center_x, front_center_y = (
                center_x + front_center_adjustment[0],
                center_y + front_center_adjustment[1],
            )

            front_center_distance = area_poly.boundary.distance(
                Point(front_center_x, front_center_y)
            )

            if front_center_distance >= 30:  # TODO: make this a parameter
                valid_solutions.append(solution)

        return valid_solutions
    def check_contain(self, length, width, center_x, center_y, area_poly):
        half_l = length / 2
        half_w = width / 2
        target_box = box(center_x - half_l, center_y - half_w, center_x + half_l, center_y + half_w)
        return area_poly.contains(target_box)



    def place_global(self, place_type, area_poly, solutions, obj_dim):
        valid_solutions = []
        minx, miny, maxx, maxy = area_poly.bounds
        width = max((maxx - minx), (maxy - miny))
        area_center = area_poly.centroid
        d_mid, d_edge = max(self.grid_size, width/5), min(width/5, self.grid_size)
        obj_length, obj_width = obj_dim[0], obj_dim[1] # NOTE, i think the width and height have been adjusted

        for solution in solutions:
            center_x, center_y = solution[0]
            # skip if out of polygon
            #if not self.check_contain(obj_length, obj_width, center_x, center_y, area_poly):
            #    continue
            # calculate distance
            center_distance = area_center.distance(Point(center_x, center_y))
            boundary_distance = area_poly.boundary.distance(Point(center_x, center_y))

            if place_type =="middle":
                solution[-1] += max((center_distance-d_mid)/d_mid, 0)

            if place_type =="edge":
                solution[-1] += max((boundary_distance-d_edge)/d_edge, 0)
                # valid_solutions.append(solution) # those are still valid solutions, but we need to move the object to the edge

            valid_solutions.append(solution)

        return valid_solutions



    def place_edge(self, area_poly, solutions, obj_dim):
        valid_solutions = []
        obj_width = obj_dim[1]
        obj_half_width = obj_width / 2

        back_center_adjustments = {
            0: (0, -obj_half_width),
            90: (-obj_half_width, 0),
            180: (0, obj_half_width),
            270: (obj_half_width, 0),
        }

        for solution in solutions:
            center_x, center_y = solution[0]
            rotation = solution[1]

            back_center_adjustment = back_center_adjustments[rotation]
            back_center_x, back_center_y = (
                center_x + back_center_adjustment[0],
                center_y + back_center_adjustment[1],
            )

            back_center_distance = area_poly.boundary.distance(
                Point(back_center_x, back_center_y)
            )
            center_distance = area_poly.boundary.distance(Point(center_x, center_y))

            if (
                back_center_distance <= self.grid_size
                and back_center_distance < center_distance
            ):
                solution[-1] += self.constraint_bouns
                # valid_solutions.append(solution) # those are still valid solutions, but we need to move the object to the edge

                # move the object to the edge
                center2back_vector = np.array(
                    [back_center_x - center_x, back_center_y - center_y]
                )
                center2back_vector /= np.linalg.norm(center2back_vector)
                offset = center2back_vector * (
                    back_center_distance + 4.5
                )  # add a small distance to avoid the object cross the wall
                solution[0] = (center_x + offset[0], center_y + offset[1])
                solution[2] = (
                    (solution[2][0][0] + offset[0], solution[2][0][1] + offset[1]),
                    (solution[2][1][0] + offset[0], solution[2][1][1] + offset[1]),
                    (solution[2][2][0] + offset[0], solution[2][2][1] + offset[1]),
                    (solution[2][3][0] + offset[0], solution[2][3][1] + offset[1]),
                )
                valid_solutions.append(solution)

        return valid_solutions

    def place_corner(self, area_poly, solutions, obj_dim):
        obj_length, obj_width = obj_dim
        obj_half_length, _ = obj_length / 2, obj_width / 2

        rotation_center_adjustments = {
            0: ((-obj_half_length, 0), (obj_half_length, 0)),
            90: ((0, obj_half_length), (0, -obj_half_length)),
            180: ((obj_half_length, 0), (-obj_half_length, 0)),
            270: ((0, -obj_half_length), (0, obj_half_length)),
        }

        edge_solutions = self.place_edge(area_poly, solutions, obj_dim)

        valid_solutions = []

        for solution in edge_solutions:
            (center_x, center_y), rotation = solution[:2]
            (dx_left, dy_left), (dx_right, dy_right) = rotation_center_adjustments[
                rotation
            ]

            left_center_x, left_center_y = center_x + dx_left, center_y + dy_left
            right_center_x, right_center_y = center_x + dx_right, center_y + dy_right

            left_center_distance = area_poly.boundary.distance(
                Point(left_center_x, left_center_y)
            )
            right_center_distance = area_poly.boundary.distance(
                Point(right_center_x, right_center_y)
            )

            if min(left_center_distance, right_center_distance) < self.grid_size:
                solution[-1] += self.constraint_bouns
                valid_solutions.append(solution)

        return valid_solutions

    def place_relative(self, area_poly, place_type, target_object, solutions):
        """
        
        """
        valid_solutions = []
        _, target_rotation, target_coords, _ = target_object
        target_polygon = Polygon(target_coords)

        min_x, min_y, max_x, max_y = target_polygon.bounds
        mean_x = (min_x + max_x) / 2
        mean_y = (min_y + max_y) / 2

        minx, miny, maxx, maxy = area_poly.bounds
        width = max((maxx - minx), (maxy - miny))

        backed_by_factor = 1
        r_low , r_high = min(width/10, self.grid_size), max(width/8, self.grid_size*3)
        comparison_dict = {
            "backed by": {
                0: lambda sol_center: sol_center[1] > max_y
                and mean_x - self.grid_size
                < sol_center[0]
                < mean_x + self.grid_size,  # in front of and centered
                90: lambda sol_center: sol_center[0] > max_x
                and mean_y - self.grid_size < sol_center[1] < mean_y + self.grid_size,
                180: lambda sol_center: sol_center[1] < min_y
                and mean_x - self.grid_size < sol_center[0] < mean_x + self.grid_size,
                270: lambda sol_center: sol_center[0] < min_x
                and mean_y - self.grid_size < sol_center[1] < mean_y + self.grid_size,
            },
            "side of": {
                0: lambda sol_center: min_y <= sol_center[1] <= max_y,
                90: lambda sol_center: min_x <= sol_center[0] <= max_x,
                180: lambda sol_center: min_y <= sol_center[1] <= max_y,
                270: lambda sol_center: min_x <= sol_center[0] <= max_x,
            },
        }


        for solution in solutions:
            sol_center = solution[0]
            distance = Point(sol_center[0], sol_center[1]).distance(Point(mean_x, mean_y))
            if place_type == "around":
                solution[-1] += max((r_low - distance), 0) + max((distance - r_high), 0)
                valid_solutions.append(solution)
            if place_type == "backed by":
                compare_func = comparison_dict.get(place_type).get(target_rotation)
                if compare_func(sol_center):
                    solution[-1] += backed_by_factor
                    valid_solutions.append(solution)

        return valid_solutions

    def place_distance(self, area_poly, distance_type, target_object, solutions):
        target_coords = target_object[2]
        target_poly = Polygon(target_coords)
        minx, miny, maxx, maxy = area_poly.bounds
        width = max((maxx - minx), (maxy - miny))

        d_near, d_far = max(self.grid_size, width/10) , max(width/8, self.grid_size*3)

        valid_solutions = []
        for solution in solutions:
            sol_coords = solution[2]
            sol_poly = Polygon(sol_coords)
            loss = 0
            distance = target_poly.distance(sol_poly)
            if distance_type == "near":
                loss = max((distance-d_near)/d_near,0)

            elif distance_type == "far":
                loss = max((d_far-distance)/d_far,0)
            solution[-1] += loss
            valid_solutions.append(solution)

        return valid_solutions

    def place_face(self, area_poly, face_type, target_object, solutions):
        if face_type == "face to":
            return self.place_face_to(target_object, solutions)

        elif face_type == "face same as":
            return self.place_face_same(target_object, solutions)

        elif face_type == "face opposite to":
            return self.place_face_opposite(target_object, solutions)

    def place_face_to(self, target_object, solutions):
        # Define unit vectors for each rotation
        unit_vectors = {
            0: np.array([0.0, 1.0]),  # Facing up
            90: np.array([1.0, 0.0]),  # Facing right
            180: np.array([0.0, -1.0]),  # Facing down
            270: np.array([-1.0, 0.0]),  # Facing left
        }

        target_coords = target_object[2]
        target_poly = Polygon(target_coords)
        face_factor = 1
        valid_solutions = []

        for solution in solutions:
            sol_center = solution[0]
            sol_rotation = solution[1]

            # Define an arbitrarily large point in the direction of the solution's rotation
            far_point = sol_center + 1e6 * unit_vectors[sol_rotation]

            # Create a half-line from the solution's center to the far point
            half_line = LineString([sol_center, far_point])

            # Check if the half-line intersects with the target polygon
            if not half_line.intersects(target_poly):
                solution[-1] += face_factor
            
            valid_solutions.append(solution)

        return valid_solutions

    def place_face_same(self, target_object, solutions):
        target_rotation = target_object[1]
        valid_solutions = []

        for solution in solutions:
            sol_rotation = solution[1]
            if sol_rotation == target_rotation:
                solution[-1] += self.constraint_bouns
                valid_solutions.append(solution)

        return valid_solutions

    def place_face_opposite(self, target_object, solutions):
        target_rotation = (target_object[1] + 180) % 360
        valid_solutions = []

        for solution in solutions:
            sol_rotation = solution[1]
            if sol_rotation == target_rotation:
                solution[-1] += self.constraint_bouns
                valid_solutions.append(solution)

        return valid_solutions

    def place_alignment_center(self, area_poly, alignment_type, target_object, solutions):
        target_center = target_object[0]
        valid_solutions = []
        eps = 5
        for solution in solutions:
            sol_center = solution[0]
            dx, dy = abs(sol_center[0] - target_center[0]), abs(sol_center[1] - target_center[1])
            if (dx > eps and dy > eps):
                solution[-1] += max((dx - eps)/dx, 0) + max((dy - eps)/dy, 0) 
            valid_solutions.append(solution)
        return valid_solutions

    def visualize_grid(self, area_poly, grid_points, solutions):
        plt.rcParams["font.family"] = "Times New Roman"
        plt.rcParams["font.size"] = 22

        # create a new figure
        fig, ax = plt.subplots()

        # draw the room
        x, y = area_poly.exterior.xy
        ax.plot(x, y, "-", label="Room", color="black", linewidth=2)

        # draw the grid points
        grid_x = [point[0] for point in grid_points]
        grid_y = [point[1] for point in grid_points]
        ax.plot(grid_x, grid_y, "o", markersize=2, color="grey")

        # draw the solutions
        for object_name, solution in solutions.items():
            center, rotation, box_coords = solution[:3]
            center_x, center_y = center

            # create a polygon for the solution
            obj_poly = Polygon(box_coords)
            x, y = obj_poly.exterior.xy
            ax.plot(x, y, "-", linewidth=2, color="black")

            # ax.text(center_x, center_y, object_name, fontsize=18, ha='center')

            # set arrow direction based on rotation
            if rotation == 0:
                ax.arrow(center_x, center_y, 0, 25, head_width=10, fc="black")
            elif rotation == 90:
                ax.arrow(center_x, center_y, 25, 0, head_width=10, fc="black")
            elif rotation == 180:
                ax.arrow(center_x, center_y, 0, -25, head_width=10, fc="black")
            elif rotation == 270:
                ax.arrow(center_x, center_y, -25, 0, head_width=10, fc="black")
        # axis off
        ax.axis("off")
        ax.set_aspect("equal", "box")  # to keep the ratios equal along x and y axis
        create_time = (
            str(datetime.datetime.now())
            .replace(" ", "-")
            .replace(":", "-")
            .replace(".", "-")
        )
        plt.savefig(f"{create_time}.pdf", bbox_inches="tight", dpi=300)
        plt.show()

    def xy2idx(self, x: float, y: float) -> Tuple[float, float]:
        """
        transfer the point position (*RL) to MAP_WH coordinate
        """
        return (x + RL) * MAP_W / ((W + 2) * RL), (y + RL) * MAP_H / ((H + 2) * RL)
    
    def get_height(self, height_map: List[List[float]], x: float, y: float) -> float:
        # note that height map is based on an extended grid (W+2)*(H+2)
        idx_x, idx_y = self.xy2idx(x, y)
        x1, y1 = int(idx_x), int(idx_y)
        x2, y2 = x1 + 1, y1 + 1
        if x1 < 0 or x2 >= MAP_W or y1 < 0 or y2 >= MAP_H:
            return WATER_HEIGHT
        xx, yy = idx_x - x1, idx_y - y1
        w1, w2, w3, w4 = (1 - xx) * (1 - yy), (1 - xx) * yy, xx * (1 - yy), xx * yy
        return max(
            min(w1 * height_map[x1][y1] + w2 * height_map[x1][y2] + w3 * height_map[x2][y1] + w4 * height_map[x2][y2], 1),
            0,
        )

class SolutionFound(Exception):
    def __init__(self, solution):
        self.solution = solution

