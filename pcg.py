from base import *
from sys_msg_en import object_selection_prompt, object_constraints_prompt
from solver import DFS_Solver
#import debugpy; debugpy.listen(('127.0.0.1', 57000)); debugpy.wait_for_client()

def find_data(name: str, data: list):
    for dic in data:
        if dic["name"] == name:
            return dic
    raise ValueError("No such data:")


def euler2quaternion(x: float, y: float, z: float) -> Tuple[float, float, float, float]:
    x, y, z = x / 2, y / 2, z / 2
    c1, s1 = cos(x), sin(x)
    c2, s2 = cos(y), sin(y)
    c3, s3 = cos(z), sin(z)
    return (
        s1 * c2 * c3 - c1 * s2 * s3,
        c1 * s2 * c3 + s1 * c2 * s3,
        c1 * c2 * s3 - s1 * s2 * c3,
        c1 * c2 * c3 + s1 * s2 * s3,
    )


def xy2idx(x: float, y: float) -> Tuple[float, float]:
    """
    transfer the point position (*RL) to MAP_WH coordinate
    """
    return (x + RL) * MAP_W / ((W + 2) * RL), (y + RL) * MAP_H / ((H + 2) * RL)


def eu_dist(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def ang_dist(a1, a2):
    a1 = (a1 % (2 * pi) + 2 * pi) % (2 * pi)
    a2 = (a2 % (2 * pi) + 2 * pi) % (2 * pi)
    return min(abs(a1 - a2), 2 * pi - abs(a1 - a2))


def p(arg1, arg2=None):
    """
    Convert point to np.array
    """
    if arg2 is None:
        return np.array(arg1)
    return np.array([arg1, arg2])


def get_height(height_map: List[List[float]], x: float, y: float) -> float:
    # note that height map is based on an extended grid (W+2)*(H+2)
    idx_x, idx_y = xy2idx(x, y)
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


def output_height_map(height_map: List[List[float]], width: int, height: int, name: str, suffix: str) -> None:
    # use cv2 to convert height_map to a width*height png
    # height_map(0,0) should be the bottom left corner, width is x, height is y
    #assert width >= height
    img = np.zeros((height, width, 3), np.uint8) # HACK, width， width is not correct
    for i in range(width):
        for j in range(height):
            grey = round(height_map[i][j] * 255)
            img[height - 1 - j][i] = (grey, grey, grey)

    outpath = "outputs/" + name + suffix + ".png"
    cv2.imwrite(outpath, img)


def output_label_map(label_map: List[List[List[float]]], width: int, height: int, name: str, suffix: str) -> None:
    # use cv2 to convert label_map to a width*height png
    # label_map(0,0) should be the bottom left corner, width is x, height is y
    #assert width >= height
    img = np.zeros((height, width, 3), np.uint8)
    for i in range(width):
        for j in range(height):
            label = label_map[i][j]
            label = [label[1] + label[2], label[3], label[4]]
            sum = label[0] + label[1] + label[2]
            if sum == 0:
                img[height - 1 - j][i] = (0, 0, 0)
            else:
                if sum > 1:
                    label = [label[0] / sum, label[1] / sum, label[2] / sum]
                lb = label[0]
                lg = label[1] / (1 - label[0]) if label[0] < 1 else 0
                lr = label[2] / (1 - label[0] - label[1]) if label[0] + label[1] < 1 else 0
                img[height - 1 - j][i] = (round(lb * 255), round(lg * 255), round(lr * 255))

    outpath = "outputs/" + name + suffix + ".png"
    cv2.imwrite(outpath, img)


def output_scene(
    all_obj_infos: Tuple[List[Tuple[float, float, float, float]], str],
    view_points: List[Tuple[float, float, float, float, float]],
    data: dict,
    garden_verse: dict,
    gen_idx: int,
    suffix: str,
) -> None:
    type2infos = {}
    for obj_infos, obj_type in all_obj_infos:
        # if len(obj_infos) == 0:
        #     continue
        if obj_type not in type2infos:
            type2infos[obj_type] = []
        type2infos[obj_type].extend(obj_infos)

    out_tree = []
    for obj_type in type2infos:
        type_dict = {}
        type_data = None
        # HACK
        for key in data:
            for dic in data[key]:
                if dic["name"] == obj_type:
                    type_data = dic
                    break
        for key in garden_verse:
            for dic in garden_verse[key]:
                if dic["name"] == obj_type:
                    type_data = dic
                    break
        type_dict["name"] = obj_type
        type_dict["path"] = type_data["path"]
        base_x, base_y, base_z = (
            type_data["cbottom"][0],
            type_data["cbottom"][1],
            type_data["cbottom"][2],
        )

        transforms = []
        for obj_info in type2infos[obj_type]:
            # print(obj_info)
            transform = {}
            forw = obj_info[3]
            scalex, scaley, scalez = 1, 1, 1
            if obj_type == "Flower_A" or obj_type == "Flower_B" or obj_type == "Flower_C" or obj_type == "Flower_D":
                rand_scale = random.uniform(1.5, 2)
                scalex = scaley = scalez = rand_scale
            elif (
                obj_type == "Lotus_A"
                or obj_type == "Lotus_B"
                or obj_type == "Lotus_Flower_A"
                or obj_type == "Lotus_Flower_B"
            ):
                rand_scale = random.uniform(1.2, 1.8)
                scalex = scaley = scalez = rand_scale
            elif obj_type == "Bamboo_A" or obj_type == "Bamboo_B" or obj_type == "Bamboo_C":
                scaley = 1.3
            elif obj_type == "Building_A" or obj_type == "Building_B" or obj_type == "Building_C" or obj_type == "Building_D":
                scaley = 1.3
            elif obj_type == "Wall_400x300":
                scaley = 1.5
            elif obj_type == "BushBig":
                rand_scale = random.uniform(0.7, 1.5)
                scalex = scalez = rand_scale
                scaley = rand_scale * random.uniform(1.6, 2.5)
            elif obj_type == "Plant_A" or obj_type == "Plant_B" or obj_type == "shaggy_soldier":
                scalex = scaley = scalez = 1.5
            elif obj_type == "SM_SquareBush":
                scaley = 1.5
            elif obj_type == "TH_Rock_A" or obj_type == "TH_Rock_B":
                rand_scale = random.uniform(2, 3)
                scalex = scaley = scalez = rand_scale
            elif obj_type == "Rock_A" or obj_type == "Rock_B" or obj_type == "Rock_C":
                rand_scale = random.uniform(7.5, 10)
                scaley = rand_scale
                scalex = scalez = 5
            elif obj_type == "hugetree":
                rand_scale = random.uniform(1.5, 2)
                scalex = scaley = scalez = rand_scale
            elif obj_type == "Bush01" or obj_type == "Bush02" or obj_type == "Bush03":
                rand_scale = random.uniform(1.5, 2)
                scalex = scaley = scalez = rand_scale
            elif obj_type == "SM_RoundBush" or obj_type == "SM_RoundBush2":
                scalex = scaley = scalez = 1.3

            rotated_x, rotated_z = base_x * cos(-forw) - base_z * sin(-forw), base_x * sin(-forw) + base_z * cos(-forw)
            transform["position"] = {
                "x": obj_info[0] + rotated_x * scalex,
                "y": obj_info[1] - base_y,
                "z": obj_info[2] - rotated_z * scalez,
            }
            quaternion = euler2quaternion(0, -forw, 0)
            transform["rotation"] = {"x": quaternion[0], "y": quaternion[1], "z": quaternion[2], "w": quaternion[3]}
            transform["scale"] = {"x": scalex, "y": scaley, "z": scalez}
            transforms.append(transform)
        type_dict["transforms"] = transforms
        out_tree.append(type_dict)

    out = {"tree": out_tree}
    out["height_map_path"] = "height_map_" + str(gen_idx) + suffix + ".png"
    out["label_map_path"] = "label_map_" + str(gen_idx) + suffix + ".png"
    out["max_height"] = MAX_HEIGHT
    out["water_height"] = WATER_HEIGHT * MAX_HEIGHT
    out["map_width"] = MAP_W
    out["map_height"] = MAP_H
    out["real_width"] = (W + 2) * RL
    out["real_height"] = (H + 2) * RL #* MAP_W / MAP_H
    out["width_offset"] = -RL
    out["height_offset"] = -RL

    out_viewpoints = []
    for i in range(len(view_points)):
        out_viewpoints.append(
            {
                "x": view_points[i][0],
                "y": view_points[i][1],
                "z": view_points[i][2],
                "xrot": view_points[i][3],
                "yrot": view_points[i][4],
            }
        )
    out["viewpoints"] = out_viewpoints

    outpath = "outputs/scene_" + str(gen_idx) + suffix + ".json"
    with open(outpath, "w") as outf:
        json.dump(out, outf)

def output_visualize(
    all_obj_infos: Tuple[List[Tuple[float, float, float, float]], str],
    data: dict,
    garden_verse: dict,
    points: List[Tuple[float, float]],
    edges: List[Tuple[List[Tuple[int, int]], List[int]]],
    areas: List[Tuple[int, List[int]]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[int, int]], dict],
    name: str,
) -> None:
    fig, ax = plt.subplots(constrained_layout=True)
    # print(areas)
    # for area in areas:
    #     combined_type, point_idxs = area
    #     if len(point_idxs) < 3:
    #         continue
    #     col = (142/255, 243/255, 171/255) if combined_type <= 3 else "white"
    #     ax.add_patch(plt.Polygon([points[idx] for idx in point_idxs], color=col, linewidth=0,))
        
    for i in range(len(edges)):
        edge_idx_list, edge_info_list = edges[i]
        for edge_idx in edge_idx_list:
            p1, p2 = points[edge_idx[0]], points[edge_idx[1]]
            # if 7 in edge_info_list:
            #     ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(238/255, 179/255, 194/255), linewidth=1) # pink
            # elif 8 in edge_info_list:
            #     ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(244/255, 248/255, 211/255), linewidth=1) # yellow
            # elif 9 in edge_info_list:
            #     ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(166/255, 214/255, 214/255), linewidth=1) # blue
            # elif 10 in edge_info_list:
            #     ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(142/255, 125/255, 190/255), linewidth=1) # purple
            # else:
            if 0 in edge_info_list or 1 in edge_info_list or 2 in edge_info_list:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(0, 0, 0), linewidth=1) # black

    ########### add buildings
    # remove empty obj_infos
    type2infos = {}
    for obj_infos, obj_type in all_obj_infos:
        if obj_type not in type2infos:
            type2infos[obj_type] = []
        type2infos[obj_type].extend(obj_infos)

    building_list = [data["pavilion"], 
                data["building"], 
                #data["wall"], 
                data["bridge"], 
                #data["corridor"],
                data["statue"],
                #garden_verse["Wall"],
                garden_verse["Pavilion"],
                garden_verse["House"],
                garden_verse["Attic"],
                garden_verse["Bridge"],
                #garden_verse["Corridor"],
                #garden_verse["Entrance"],
    ]
    building_info_dict = {} # building_name : building_info{}, to get the size info, HACK just add size info
    for building_class in building_list:
        for building_object in building_class:
            building_info_dict[building_object["name"]] = building_object

    for obj_type in type2infos:
        if obj_type in building_info_dict:
            for obj_info in type2infos[obj_type]:
                posX, posZ, forw = obj_info[0], obj_info[2], obj_info[3]
                size_width, size_height = building_info_dict[obj_type]["size"][0], building_info_dict[obj_type]["size"][2]
                ax.add_patch(patches.Rectangle((posX-size_width/2, posZ-size_height/2), size_width, size_height, angle=forw/np.pi*180, linewidth=1, edgecolor="black", facecolor='white')) # lefdown posX, posZ, width, height
    ########### add trees

    plt.axis("off")
    plt.savefig("outputs/" + name + ".png")
    plt.close()
    plt.clf()

def output_json(object_selection: dict, object_constraints: dict, text: str):

    obj_outpath = "outputs/obj_res.json"
    con_outpath = "outputs/con_res.json"
    with open(obj_outpath, "w") as outf:
        json.dump(object_selection, outf, indent=4)
    with open(con_outpath, "w") as outf:
        json.dump(object_constraints, outf, indent=4)

def pathway_score(
    all_obj_infos: Tuple[List[Tuple[float, float, float, float]], str],
    data: dict,
    garden_verse: dict,
    points: List[Tuple[float, float]],
    edges: List[Tuple[List[Tuple[int, int]], List[int]]],
    areas: List[Tuple[int, List[int]]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[int, int]], dict],
    name: str,
) -> None:

    pathway_score = 0
    scoring_object_number = 0
    ########### add buildings
    # remove empty obj_infos
    type2infos = {}
    for obj_infos, obj_type in all_obj_infos:
        if obj_type not in type2infos:
            type2infos[obj_type] = []
        type2infos[obj_type].extend(obj_infos)

    building_list = [data["pavilion"], 
                data["building"], 
                #data["wall"], 
                #data["bridge"], 
                #data["corridor"],
                data["statue"],
                garden_verse["Pavilion"],
                garden_verse["House"],
                garden_verse["Attic"],
        ]
    building_info_dict = {} # building_name : building_info{}, to get the size info, HACK just add size info
    for building_class in building_list:
        for building_object in building_class:
            building_info_dict[building_object["name"]] = building_object

    for obj_type in type2infos:
        if obj_type in building_info_dict:
            for obj_info in type2infos[obj_type]:
                # get information from obj_info and building_info_dict
                posX, posZ, forw = obj_info[0], obj_info[2], obj_info[3]
                size_length, size_width = building_info_dict[obj_type]["size"][0], building_info_dict[obj_type]["size"][2]

                # calculate distance between EDGE and important objects
                for edge_group in edges:
                    edge_idxs, info = edge_group
                    for i in range(len(edge_idxs) - 1):
                        if edge_idxs[i][1] != edge_idxs[i + 1][0]:
                            raise ValueError("Edge not connected")
                    point_idxs = [idx[0] for idx in edge_idxs] + [edge_idxs[-1][1]] # collect all points' indices
                    point_locs = [points[idx] for idx in point_idxs]
                    line = LineString(point_locs)
                    distance = line.distance(Point(posX, posZ))
                    if distance < 2:
                        pathway_score +=1
                        break
                scoring_object_number +=1
    normalized_score = 0
    if scoring_object_number != 0:
        normalized_score = pathway_score/scoring_object_number

    pathway = {"pathway score": pathway_score,
            "number": scoring_object_number,
            "normalized score": normalized_score
    }
    con_outpath = "outputs/pathway.json"

    with open(con_outpath, "w") as outf:
        json.dump(pathway, outf, indent=4)


def random_placing(
    poly_: Polygon, size_list: List[Tuple[float, float]], ratio: float, override: bool, buffer: float = 0
) -> List[Tuple[float, float, int]]:
    """
    randomly place objects in the polygon
    """
    poly = poly_.buffer(-buffer) if buffer > 0 else deepcopy(poly_)
    if poly.area<0.1:
        return []
    target_area = poly.area * ratio
    now_area = 0
    type_areas = [size[0] * size[1] for size in size_list]
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    num_types = len(size_list)

    continuous_fail = 0
    while True:
        typ = random.randint(0, num_types - 1)
        x = random.uniform(minx, maxx)
        z = random.uniform(minz, maxz)
        if poly.contains(Point(x, z)):
            valid = True
            if not override:
                for px, pz, p_typ in point_and_types:
                    if (
                        abs(px - x) < (size_list[p_typ][0] + size_list[typ][0]) / 2
                        and abs(pz - z) < (size_list[p_typ][1] + size_list[typ][1]) / 2
                    ):
                        valid = False
                        break
            if valid:
                point_and_types.append((x, z, typ))
                now_area += type_areas[typ]
                continuous_fail = 0
                if now_area >= target_area:
                    break
            else:
                continuous_fail += 1
                if continuous_fail > 50:
                    override = True
    return point_and_types

def area_aware_random_placing(
    poly_: Polygon, size_list: List[Tuple[float, float]], ratio: float, override: bool, buffer: float = 0
) -> List[Tuple[float, float, int]]:
    """
    randomly place objects in the polygon
    """
    poly = poly_.buffer(-buffer) if buffer > 0 else deepcopy(poly_)
    if poly.area<0.1:
        return []
    target_area = poly.area * ratio
    now_area = 0
    type_areas = [size[0] * size[1] for size in size_list]
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    num_types = len(size_list)

    continuous_fail = 0
    for i in range(1000):
        typ = random.randint(0, num_types - 1)
        x = random.uniform(minx, maxx)
        z = random.uniform(minz, maxz)
        length, width = size_list[typ][0], size_list[typ][1]
        half_length = length / 2
        half_width = width / 2
        
        # Define corners (counter-clockwise)
        corners = [
            (x - half_length, z - half_width),  # bottom-left
            (x + half_length, z - half_width),  # bottom-right
            (x + half_length, z + half_width),  # top-right
            (x - half_length, z + half_width)   # top-left
        ]
        
        # Create the polygon
        polygon = Polygon(corners)

        if poly.contains(polygon):
            valid = True
            if not override: # collision
                for px, pz, p_typ in point_and_types:
                    if (
                        abs(px - x) < (size_list[p_typ][0] + size_list[typ][0]) / 2
                        and abs(pz - z) < (size_list[p_typ][1] + size_list[typ][1]) / 2
                    ):
                        valid = False
                        break
            if valid:
                point_and_types.append((x, z, typ))
                now_area += type_areas[typ]
                continuous_fail = 0
                if now_area >= target_area:
                    break
            else:
                continuous_fail += 1
                if continuous_fail > 50:
                    override = True
    return point_and_types


def dense_random_placing(
    poly_: Polygon, size_list: List[Tuple[float, float]], ratio: float, override: bool, buffer: float = 0
) -> List[Tuple[float, float, int]]:
    """
    randomly place objects in the polygon
    """
    poly = poly_.buffer(-buffer) if buffer > 0 else deepcopy(poly_)
    if poly.area<0.1:
        return []
    target_area = poly.area * ratio
    now_area = 0
    type_areas = [size[0] * size[1] for size in size_list]
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    num_types = len(size_list)

    continuous_fail = 0
    while True:
        typ = random.randint(0, num_types - 1)
        x = random.uniform(minx, maxx)
        z = random.uniform(minz, maxz)
        if poly.contains(Point(x, z)):
            valid = True
            if not override:
                for px, pz, p_typ in point_and_types:
                    if (
                        abs(px - x) < (size_list[p_typ][0] + size_list[typ][0]) / 2
                        and abs(pz - z) < (size_list[p_typ][1] + size_list[typ][1]) / 2
                    ):
                        valid = False
                        break
            if valid:
                point_and_types.append((x, z, typ))
                now_area += type_areas[typ]
                continuous_fail = 0
                if now_area >= target_area:
                    break
            else:
                continuous_fail += 1
                if continuous_fail > 50:
                    override = True
    return point_and_types

def grid_random_placing(
    poly_: Polygon, size_list: List[Tuple[float, float]], gx: float, gz: float
) -> List[Tuple[float, float, int]]:
    """
    gx, gz: size of the grid
    randomly place objects in each grid
    """
    poly = deepcopy(poly_)
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    wnum, hnum = int((maxx - minx) / gx), int((maxz - minz) / gz)
    if wnum == 0 or hnum == 0:
        return []
    gx, gz = (maxx - minx) / wnum, (maxz - minz) / hnum
    num_types = len(size_list)
    for i in range(wnum):
        for j in range(hnum):
            typ = random.randint(0, num_types - 1)
            xl, xh, zl, zh = minx + i * gx, minx + (i + 1) * gx, minz + j * gz, minz + (j + 1) * gz
            mx, mz = size_list[typ][0], size_list[typ][1]
            for k in range(10):
                x, z = random.uniform(xl + mx / 2, xh - mx / 2), random.uniform(zl + mz / 2, zh - mz / 2)
                if xh - xl <= mx:
                    x = (xh + xl) / 2
                if zh - zl <= mz:
                    z = (zh + zl) / 2
                if poly.contains(Point(x, z)):
                    point_and_types.append((x, z, typ))
                    break
    return point_and_types


def group_random_placing(
    poly_: Polygon,
    size_list: List[Tuple[float, float]],
    ratio: float,
    buffer: float,
    group_range: float,
    group_num: int,
) -> List[Tuple[float, float, int]]:
    """
    randomly place objects in the form of group
    """
    poly = poly_.buffer(-group_range - buffer)
    target_area = poly.area * ratio
    now_area = 0
    type_areas = [size[0] * size[1] for size in size_list]
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    num_types = len(size_list)

    while True:
        typ = random.randint(0, num_types - 1)
        x = random.uniform(minx, maxx)
        z = random.uniform(minz, maxz)
        if poly.contains(Point(x, z)):
            for i in range(group_num):
                rand_angle = random.uniform(0, 2 * pi)
                rand_radius = random.uniform(0, group_range)
                new_x = x + rand_radius * cos(rand_angle)
                new_z = z + rand_radius * sin(rand_angle)
                point_and_types.append((new_x, new_z, typ))
                now_area += type_areas[typ]
                if now_area >= target_area:
                    break
        if now_area >= target_area:
            break

    return point_and_types


def maze_random_placing(
    poly_: Polygon, size_list: List[Tuple[float, float]], gx: float, gz: float, ratio: float
) -> List[Tuple[float, float, int]]:
    """
    gx, gz: size of the grid
    randomly generate a maze and place objects (not exceeding ratio)
    """
    poly = deepcopy(poly_)
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    wnum, hnum = int((maxx - minx) / gx), int((maxz - minz) / gz)
    if wnum == 0 or hnum == 0:
        return []
    gx, gz = (maxx - minx) / wnum, (maxz - minz) / hnum
    target_area = poly.area * ratio
    target_num = int(target_area / (gx * gz)) + 1
    num_types = len(size_list)

    road = set()
    dxy = [0, 2, 0, -2, 0]

    def maze_dfs(curr_pos):
        road.add(curr_pos)
        stack = [curr_pos]
        while stack:
            curr_pos = stack.pop()
            p = [0, 1, 2, 3]
            random.shuffle(p)
            for i in p:
                next_pos = (curr_pos[0] + dxy[i], curr_pos[1] + dxy[i + 1])
                if 0 <= next_pos[0] < wnum and 0 <= next_pos[1] < hnum and next_pos not in road:
                    road.add(((curr_pos[0] + next_pos[0]) // 2, (curr_pos[1] + next_pos[1]) // 2))
                    road.add(next_pos)
                    stack.append(next_pos)

    maze_dfs((0, 0))

    candidates = []
    for i in range(wnum):
        for j in range(hnum):
            x, z = (i + 0.5) * gx + minx, (j + 0.5) * gz + minz
            if poly.contains(Point(x, z)) and ((i, j) not in road):
                candidates.append((x, z))
    random.shuffle(candidates)
    for i in range(min(target_num, len(candidates))):
        typ = random.randint(0, num_types - 1)
        point_and_types.append((candidates[i][0], candidates[i][1], typ))
    return point_and_types


def along_bordered_placing(poly_: Polygon, mx: float, mz: float, step: float, layers: int) -> List[Tuple[float, float]]:
    """
    mx, mz: boundbox of the object
    step: distance between two objects
    layers: number of layers
    place objects along the border of the polygon
    """
    poly = deepcopy(poly_)
    points = []
    for k in range(layers):
        if k == 0:
            poly = poly.buffer(-max(mx, mz) / 2)
        else:
            poly = poly.buffer(-step)
        if not isinstance(poly, Polygon):
            break
        boundary_ring = poly.exterior
        layer_num = max(int(boundary_ring.length / step) - 1, 1)
        for i in range(layer_num):
            point = boundary_ring.interpolate(i * step)
            points.append((point.x, point.y))
    return points


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


def along_line_placing(
    point_locs: List[Tuple[float, float]], step: float, width: float
) -> List[Tuple[float, float, float]]:
    loc_and_forws = []
    last_idx, last_loc, last_forw = 0, p(deepcopy(point_locs[0])), None

    while True:
        next_idx = None
        # Find next point that's further than step distance
        for i in range(last_idx, len(point_locs)):
            next_idx = i
            if eu_dist(last_loc, point_locs[i]) > step:
                break
        # Stop if the whole edge is too short
        if eu_dist(last_loc, point_locs[next_idx]) <= 0.25:
            break
        # Get Direction
        forw_vec = (point_locs[next_idx][0] - last_loc[0], point_locs[next_idx][1] - last_loc[1]) # (x, y)
        forw = np.arctan2(forw_vec[1], forw_vec[0]) # angle
        # Decide Position
        sp, ep = None, None # start_pos, end_pos
        if last_forw is None:
            sp, ep = last_loc, last_loc + step * p(cos(forw), sin(forw))
        else:
            a_dist = ang_dist(forw, last_forw)
            if eu_dist(last_loc, point_locs[next_idx]) + sin(a_dist / 2) * width * 0.75 < step:
                ep = point_locs[next_idx]
                sp = ep - step * p(cos(forw), sin(forw))
            else:
                sp = last_loc - sin(a_dist / 2) * width * p(cos(forw), sin(forw)) * 0.75
                ep = sp + step * p(cos(forw), sin(forw))

        midx, midy = (sp[0] + ep[0]) / 2, (sp[1] + ep[1]) / 2
        loc_and_forws.append((midx, midy, forw))
        # break if reaching the end of the path
        xl, yl, xh, yh = (
            min(sp[0], ep[0]) - 1e-3,
            min(sp[1], ep[1]) - 1e-3,
            max(sp[0], ep[0]) + 1e-3,
            max(sp[1], ep[1]) + 1e-3,
        )
        xlast, ylast = point_locs[-1][0], point_locs[-1][1]
        if xl <= xlast <= xh and yl <= ylast <= yh:
            break
        last_loc, last_forw, last_idx = ep, forw, next_idx

    return loc_and_forws

def along_line_corridor_placing(
    point_locs: List[Tuple[float, float]], step: float, width: float
) -> List[Tuple[float, float, float]]:
    loc_and_forws = []
    last_idx, last_loc, last_forw = 0, p(deepcopy(point_locs[0])), None

    while True:
        next_idx = None
        # Find next point that's further than step distance
        for i in range(last_idx, len(point_locs)):
            next_idx = i
            if eu_dist(last_loc, point_locs[i]) > step:
                break
        # Stop if the whole edge is too short
        if eu_dist(last_loc, point_locs[next_idx]) <= 0.25:
            break
        # Get Direction
        forw_vec = (point_locs[next_idx][0] - last_loc[0], point_locs[next_idx][1] - last_loc[1]) # (x, y)
        forw = np.arctan2(forw_vec[1], forw_vec[0]) # angle
        # Decide Position
        sp, ep = None, None # start_pos, end_pos
        if last_forw is None:
            sp, ep = last_loc, last_loc + step * p(cos(forw), sin(forw))
        else:
            a_dist = ang_dist(forw, last_forw)
            if eu_dist(last_loc, point_locs[next_idx]) + sin(a_dist / 2) * width * 0.75 < step:
               ep = point_locs[next_idx]
               sp = ep - step * p(cos(forw), sin(forw))
            else:
                ## cancel curve adjustment
                sp = last_loc - sin(a_dist / 2) * width * p(cos(forw), sin(forw)) * 0.75
                ep = sp + step * p(cos(forw), sin(forw))
            #sp, ep = last_loc, last_loc + step * p(cos(forw), sin(forw))

        midx, midy = (sp[0] + ep[0]) / 2, (sp[1] + ep[1]) / 2
        loc_and_forws.append((midx, midy, forw))
        # break if reaching the end of the path
        xl, yl, xh, yh = (
            min(sp[0], ep[0]) - 1e-3,
            min(sp[1], ep[1]) - 1e-3,
            max(sp[0], ep[0]) + 1e-3,
            max(sp[1], ep[1]) + 1e-3,
        )
        xlast, ylast = point_locs[-1][0], point_locs[-1][1]
        if xl <= xlast <= xh and yl <= ylast <= yh:
            break
        last_loc, last_forw, last_idx = ep, forw, next_idx

    return loc_and_forws

def along_line_rock_placing(
    point_locs: List[Tuple[float, float]], size_list: List[Tuple[float, float]], info: List[float], height_map: List[List[float]]
) -> List[Tuple[float, float, float, float, int]]:
    
    weights = [0.25, 0.1, 0.1, 0.25, 0.25, 0.05]
    num_types = len(size_list) 
    types_list = list(range(num_types))
    
    loc_forws_typ = []
    last_idx, last_loc, last_forw = 0, p(deepcopy(point_locs[0])), None

    while True:
        #typ = random.randint(0, num_types - 1) # object_type
        typ = np.random.choice(types_list, p=weights)
        step, width = size_list[typ][0], size_list[typ][1]

        next_idx = None
        # Find next point that's further than step distance
        for i in range(last_idx, len(point_locs)):
            next_idx = i
            if eu_dist(last_loc, point_locs[i]) > step:
                break
        # Stop if the whole edge is too short
        if eu_dist(last_loc, point_locs[next_idx]) <= 0.25:
            break
        # Get Direction
        forw_vec = (point_locs[next_idx][0] - last_loc[0], point_locs[next_idx][1] - last_loc[1]) # (x, y)
        forw = np.arctan2(forw_vec[1], forw_vec[0]) # angle, [-pi, pi]
        # Decide Position
        sp, ep = None, None # start_pos, end_pos
        if last_forw is None:
            sp, ep = last_loc, last_loc + step * p(cos(forw), sin(forw))
        else:
            a_dist = ang_dist(forw, last_forw)
            if eu_dist(last_loc, point_locs[next_idx]) + sin(a_dist / 2) * width * 0.75 < step:
               ep = point_locs[next_idx]
               sp = ep - step * p(cos(forw), sin(forw))
            else:
                ## cancel curve adjustment
                sp = last_loc - sin(a_dist / 2) * width * p(cos(forw), sin(forw)) * 0.75
                ep = sp + step * p(cos(forw), sin(forw))
            #sp, ep = last_loc, last_loc + step * p(cos(forw), sin(forw))

        midx, midy = (sp[0] + ep[0]) / 2, (sp[1] + ep[1]) / 2
        # adjust forw according to waterside direction
        if 7 in info:
            if forw >= (np.pi/2) and forw <= (-np.pi/2) :
                loc_forws_typ.append((midx, 0, midy, forw-np.pi, typ))
            else:
                loc_forws_typ.append((midx, 0, midy, forw, typ))
        if 9 in info:
            if forw <= (np.pi/2) and forw >= (-np.pi/2) :
                loc_forws_typ.append((midx, 0, midy, forw+np.pi, typ))
            else:
                loc_forws_typ.append((midx, 0, midy, forw, typ))
        if 8 in info:
            if forw >= -np.pi and forw <= 0 :
                loc_forws_typ.append((midx, 0, midy, forw+np.pi, typ))
            else:
                loc_forws_typ.append((midx, 0, midy, forw, typ))
        if 10 in info:
            if forw <= np.pi and forw >= 0 :
                loc_forws_typ.append((midx, 0, midy, forw-np.pi, typ))
            else:
                loc_forws_typ.append((midx, 0, midy, forw, typ))
        # break if reaching the end of the path
        xl, yl, xh, yh = (
            min(sp[0], ep[0]) - 1e-3,
            min(sp[1], ep[1]) - 1e-3,
            max(sp[0], ep[0]) + 1e-3,
            max(sp[1], ep[1]) + 1e-3,
        )
        xlast, ylast = point_locs[-1][0], point_locs[-1][1]
        if xl <= xlast <= xh and yl <= ylast <= yh:
            break
        last_loc, last_forw, last_idx = ep, forw, next_idx
    
    # assign height
    start_obj, end_obj = loc_forws_typ[0], loc_forws_typ[-1]
    sp = p(start_obj[0], start_obj[1]) - step / 2 * p(cos(start_obj[2]), sin(start_obj[2]))
    ep = p(end_obj[0], end_obj[1]) + step / 2 * p(cos(end_obj[2]), sin(end_obj[2]))
    rock_height =  max(min(get_height(height_map, sp[0], sp[1]), get_height(height_map, ep[0], ep[1])), 0.093)
    base_height = rock_height * MAX_HEIGHT # + random() * 0.01, a trick to get 
    
    loc_forws_typ_list = []
    for i in range(len(loc_forws_typ)):
        x, y, z, rotation, type = loc_forws_typ[i]
        loc_forws_typ_list.append((x, base_height, z, rotation, type)) # tuple element in python can't be assigned alone

    return loc_forws_typ_list


def build_zigzagbridge(
    height_map: List[List[float]], point_locs: List[Tuple[float, float]], data: dict
) -> List[Tuple[float, float, float, float]]:
    step, height, width = data["size"][0], data["size"][1], data["size"][2]
    loc_and_forws = along_line_placing(point_locs, step, width) # [(midx, midy, forw),...]
    start_obj, end_obj = loc_and_forws[0], loc_and_forws[-1]
    sp = p(start_obj[0], start_obj[1]) - step / 2 * p(cos(start_obj[2]), sin(start_obj[2]))
    ep = p(end_obj[0], end_obj[1]) + step / 2 * p(cos(end_obj[2]), sin(end_obj[2]))
    lakeside_h = max(min(get_height(height_map, sp[0], sp[1]), get_height(height_map, ep[0], ep[1])), 0.1)
    base_height = lakeside_h * MAX_HEIGHT - (height - 1)

    obj_infos = []
    for loc_and_forw in loc_and_forws:
        obj_infos.append((loc_and_forw[0], base_height, loc_and_forw[1], loc_and_forw[2])) # X, Y, Z
    return obj_infos


def build_bridge(
    height_map: List[List[float]], point_locs: List[Tuple[float, float]], data: dict
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int]]]:
    length = data["size"][0] # data length
    # get edge length
    edge_length = 0 
    for i in range(len(point_locs) - 1):
        edge_length += eu_dist(point_locs[i], point_locs[i + 1])
    # get the index of middle point
    mid_idx, now_length = -1, 0
    for i in range(len(point_locs) - 1):
        if now_length / edge_length >= 0.5:
            mid_idx = i
            break
        now_length += eu_dist(point_locs[i], point_locs[i + 1])
    idx1, idx2 = deepcopy(mid_idx), deepcopy(mid_idx)
    while True:
        if idx1 > 0:
            idx1 -= 1
        if eu_dist(point_locs[idx1], point_locs[idx2]) >= length:
            break
        if idx2 < len(point_locs) - 1:
            idx2 += 1
        if eu_dist(point_locs[idx1], point_locs[idx2]) >= length:
            break
        if idx1 == 0 and idx2 == (len(point_locs)-1) and eu_dist(point_locs[idx1], point_locs[idx2]) < length:
            return -1, -1 # if the bridge can not locate in the road because of the size
    sp, ep = p(point_locs[idx1]), p(point_locs[idx2]) # start_point, end_point
    midp, forw = (sp + ep) / 2, np.arctan2(ep[1] - sp[1], ep[0] - sp[0])

    sp, ep = midp - length / 2 * p(cos(forw), sin(forw)), midp + length / 2 * p(cos(forw), sin(forw))
    poly1, poly2 = None, None
    edges1 = [(point_locs[i], point_locs[i + 1]) for i in range(idx1)] + [(point_locs[idx1], sp)]
    edges2 = [(ep, point_locs[idx2])] + [(point_locs[i], point_locs[i + 1]) for i in range(idx2, len(point_locs) - 1)]
    for edge in edges1:
        p1, p2 = p(edge[0]), p(edge[1])
        tang = p(p2[0] - p1[0], p2[1] - p1[1])
        tang = tang / np.linalg.norm(tang)
        norm = p(p2[1] - p1[1], p1[0] - p2[0])
        norm = norm / np.linalg.norm(norm)
        poly = Polygon(
            [
                p1 + norm * MAIN_ROAD_WIDTH / 2 - tang,
                p1 - norm * MAIN_ROAD_WIDTH / 2 - tang,
                p2 - norm * MAIN_ROAD_WIDTH / 2 + tang,
                p2 + norm * MAIN_ROAD_WIDTH / 2 + tang,
            ]
        )
        if poly1 is None:
            poly1 = poly
        else:
            poly1 = poly1.union(poly)
    for edge in edges2:
        p1, p2 = p(edge[0]), p(edge[1])
        tang = p(p2[0] - p1[0], p2[1] - p1[1])
        tang = tang / np.linalg.norm(tang)
        norm = p(p2[1] - p1[1], p1[0] - p2[0])
        norm = norm / np.linalg.norm(norm)
        poly = Polygon(
            [
                p1 + norm * MAIN_ROAD_WIDTH / 2 - tang,
                p1 - norm * MAIN_ROAD_WIDTH / 2 - tang,
                p2 - norm * MAIN_ROAD_WIDTH / 2 + tang,
                p2 + norm * MAIN_ROAD_WIDTH / 2 + tang,
            ]
        )
        if poly2 is None:
            poly2 = poly
        else:
            poly2 = poly2.union(poly)
    poly1, poly2 = poly1.simplify(0, False), poly2.simplify(0, False)

    base_height_ratio = 0.1
    road_idxs = override_height(height_map, poly1, base_height_ratio, 3) + override_height(
        height_map, poly2, base_height_ratio, 3
    )
    obj_infos = [(midp[0], base_height_ratio * MAX_HEIGHT - 1.4, midp[1], forw)]
    return obj_infos, road_idxs

def build_shortCorridor(
    height_map: List[List[float]], point_locs: List[Tuple[float, float]], data: dict
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int]]]:
    step, height, width = data["size"][0], data["size"][1], data["size"][2]
    loc_and_forws = along_line_corridor_placing(point_locs, step, width)
    start_obj, end_obj = loc_and_forws[0], loc_and_forws[-1]
    sp = p(start_obj[0], start_obj[1]) - step / 2 * p(cos(start_obj[2]), sin(start_obj[2]))
    ep = p(end_obj[0], end_obj[1]) + step / 2 * p(cos(end_obj[2]), sin(end_obj[2]))
    corridor_height = min(get_height(height_map, sp[0], sp[1]), get_height(height_map, ep[0], ep[1]))
    base_height = corridor_height * MAX_HEIGHT

    obj_infos = []
    for loc_and_forw in loc_and_forws:
        obj_infos.append((loc_and_forw[0], base_height, loc_and_forw[1], loc_and_forw[2])) # X, Y, Z, rotation angle
    return obj_infos

def build_lakeside_rock(
    height_map: List[List[float]], point_locs: List[Tuple[float, float]], datas: dict, info: List[float]
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int]]]:
    """
    add waterside rock with different weights, A: 0.3; B: 0.1; C: 0.1; D: 0.3; E: 0.25; F: 0.05;
    """
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas] # X,Z; step, width


    #step, height, width = data["size"][0], data["size"][1], data["size"][2]
    loc_forws_typ_list = along_line_rock_placing(point_locs, size_list, info, height_map)
    # start_obj, end_obj = loc_forws_typ[0], loc_forws_typ[-1]
    # sp = p(start_obj[0], start_obj[1]) - step / 2 * p(cos(start_obj[2]), sin(start_obj[2]))
    # ep = p(end_obj[0], end_obj[1]) + step / 2 * p(cos(end_obj[2]), sin(end_obj[2]))
    # rock_height = min(get_height(height_map, sp[0], sp[1]), get_height(height_map, ep[0], ep[1]))
    # base_height = rock_height * MAX_HEIGHT #+ random() * 0.01

    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for loc_forws_typ in loc_forws_typ_list:
        x, y, z, rotation, typ = loc_forws_typ
        obj_infos_dict[typ].append((x, y, z, rotation))
    
    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))

    return obj_infos_list



def build_wall(
    height_map: List[List[float]], point_locs: List[Tuple[float, float]], data: dict
) -> List[Tuple[float, float, float, float]]:
    step, width = 4, data["size"][2]
    locs_and_forws = along_line_placing(point_locs, step, width)

    obj_infos = []
    for loc_and_forw in locs_and_forws:
        midp, forw = p(loc_and_forw[0], loc_and_forw[1]), loc_and_forw[2]
        sp, ep = midp - step / 2 * p(cos(forw), sin(forw)), midp + step / 2 * p(cos(forw), sin(forw))
        h1, h2 = get_height(height_map, sp[0], sp[1]), get_height(height_map, ep[0], ep[1])
        obj_infos.append((midp[0], min(h1, h2) * MAX_HEIGHT, midp[1], forw))
    return obj_infos


def build_entrance(
    height_map: List[List[float]],
    point_locs: List[Tuple[float, float]],
    data: dict,
    entrance_points: List[Tuple[float, float]],
) -> List[Tuple[float, float, float, float]]:
    step, width = 4, data["size"][2]
    locs_and_forws = along_line_placing(point_locs, step, width)

    obj_infos = []
    for loc_and_forw in locs_and_forws:
        midp, forw = p(loc_and_forw[0], loc_and_forw[1]), loc_and_forw[2]
        far_from_entrance = True
        for entrance_point in entrance_points:
            if eu_dist(midp, entrance_point) < 5:
                far_from_entrance = False
                break
        if far_from_entrance:
            sp, ep = midp - step / 2 * p(cos(forw), sin(forw)), midp + step / 2 * p(cos(forw), sin(forw))
            h1, h2 = get_height(height_map, sp[0], sp[1]), get_height(height_map, ep[0], ep[1])
            obj_infos.append((midp[0], min(h1, h2) * MAX_HEIGHT, midp[1], forw))
    return obj_infos


def add_lotus(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas] # X,Z
    if poly.area > 4000:
        return []
    point_and_types = random_placing(poly, size_list, 0.6, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    model_height = WATER_HEIGHT * MAX_HEIGHT - 0.05 -1
    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        if get_height(height_map, x, z) < WATER_HEIGHT - 0.005:
            obj_infos_dict[typ].append((x, model_height, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list

def add_lotus_group(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas] # X,Z
    if poly.area > 4000:
        return []
    lotus_ratio = 0.4  
    random_number = random.randint(1, 3)
    if random_number == 1 : lotus_ratio = 0.5
    elif random_number == 2 : lotus_ratio = 0.4
    else : lotus_ratio = 0.3
    point_and_types = random_placing(poly, size_list, lotus_ratio, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    model_height = WATER_HEIGHT * MAX_HEIGHT - 0.05 -1
    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        if get_height(height_map, x, z) < WATER_HEIGHT - 0.005:
            obj_infos_dict[typ].append((x, model_height, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list

def add_lakerock(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    point_and_types = random_placing(poly, size_list, 0.001, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    base_height = WATER_HEIGHT * MAX_HEIGHT - 0.6
    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        if get_height(height_map, x, z) < WATER_HEIGHT - 0.005:
            obj_infos_dict[typ].append((x, base_height + random.uniform(-0.2, 0.2), z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_rockmaze(
    height_map: List[List[float]],
    poly: Polygon,
    hill_rock_data: List[dict],
    lake_rock_data: List[dict],
    tree_data: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    hillrock_size_list = [(dat["size"][0], dat["size"][2]) for dat in hill_rock_data]
    # lakerock_size_list = [(dat["size"][0], dat["size"][2]) for dat in lake_rock_data]
    tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_data]
    # lakerock_point_and_types = maze_random_placing(poly, lakerock_size_list, 3, 3, 0.02)
    hillrock_point_and_types = group_random_placing(poly, hillrock_size_list, 0.004, 2, 2, 3)
    tree_point_and_types = random_placing(poly, tree_size_list, 0.2, True)
    hillrock_obj_infos_dict, lakerock_obj_infos_dict, tree_obj_infos_dict = {}, {}, {}
    for typ_idx in range(len(hill_rock_data)):
        hillrock_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(lake_rock_data)):
        lakerock_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(tree_data)):
        tree_obj_infos_dict[typ_idx] = []

    for point_and_type in hillrock_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        hillrock_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    # for point_and_type in lakerock_point_and_types:
    #     x, z, typ = point_and_type
    #     x += random.uniform(-1, 1)
    #     z += random.uniform(-1, 1)
    #     y = get_height(height_map, x, z) * MAX_HEIGHT
    #     lakerock_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in tree_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        tree_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    obj_infos_list = []
    for typ_idx in hillrock_obj_infos_dict:
        obj_infos_list.append((hillrock_obj_infos_dict[typ_idx], hill_rock_data[typ_idx]["name"]))
    for typ_idx in lakerock_obj_infos_dict:
        obj_infos_list.append((lakerock_obj_infos_dict[typ_idx], lake_rock_data[typ_idx]["name"]))
    for typ_idx in tree_obj_infos_dict:
        obj_infos_list.append((tree_obj_infos_dict[typ_idx], tree_data[typ_idx]["name"]))
    return obj_infos_list


def add_bushes(
    height_map: List[List[float]], poly: Polygon, rectbush_datas: List[dict], bigbush_datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    rectbush_size_list = [(dat["size"][0], dat["size"][2]) for dat in rectbush_datas]
    bigbush_size_list = [(dat["size"][0], dat["size"][2]) for dat in bigbush_datas]
    poly_area = poly.area
    point_and_types = None
    typeflag = 0
    if poly_area < 500:
        point_and_types = grid_random_placing(poly, rectbush_size_list, 2, 2)
    else:
        if poly_area < 2000 and random.random() < 0.7:
            point_and_types = maze_random_placing(poly, rectbush_size_list, 2, 2, 0.45)
        else:
            typeflag = 1
            point_and_types = random_placing(poly, bigbush_size_list, 0.3, True, 3)
    obj_infos_dict = {}
    length = len(rectbush_datas) if typeflag == 0 else len(bigbush_datas)
    for typ_idx in range(length):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        if typeflag == 0:
            obj_infos_list.append((obj_infos_dict[typ_idx], rectbush_datas[typ_idx]["name"]))
        else:
            obj_infos_list.append((obj_infos_dict[typ_idx], bigbush_datas[typ_idx]["name"]))
    return obj_infos_list


def add_few_trees(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    point_and_types = random_placing(poly, size_list, 0.2, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_bamboos(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    poly_area = poly.area
    point_and_types = None
    if poly_area < 1000:
        point_and_types = grid_random_placing(poly, size_list, 2, 2)
    else:
        point_and_types = random_placing(poly, size_list, 0.6, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list

def add_bamboos_gv(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    poly_area = poly.area
    point_and_types = None
    point_and_types = dense_random_placing(poly, size_list, 0.6, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_trees(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    poly_area = poly.area
    point_and_types = random_placing(poly, size_list, 0.6, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list

def add_trees_gv(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    tree_autum_winter_list = ["Acer_buergerianum_A", 
        "Camellia_combination_B", 
        "Camellia_combination_C",
        "Maple_combination_B",
        "Maple_combination_C",
        "Wintersweet_rock_combination_A",
        "Wintersweet_rock_combination_B",
    ]
    tree_list = ["Abelia_grandiflora_stone_A",
        "Asiaticapple_combination_A",
        "Camellia_combination_A", 
        "Cinnamomum_combination_A", 
        "Hydrangea_camellia_rock_combination_A",
        "Magnolia_rock_combination_A",
        "Magnolia_rock_combination_B",
        "Osmanthus_flower_combination_A",
        "Osmanthus_trees_combination_A",
        "Osmanthus_tree_combination_B",
        "Paraso_flower_rock_combination_A",
        "Peach_combination_A",
        "Peach_combination_B",
        "Pine_rock_combination_A",
        "Rock_Pine_combination_A",
        "Cinnamomum_maple_combination_A",
        "Sakura_combination_A",
        "Sakura_combination_B",
        "Sakura_rock_combination_A",
    ]
    # HACK, it is not reasonable for
    tree_data_list = []
    for name in tree_list:
        tree_data_list.append(find_data(name, datas))

    size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_data_list]
    poly_area = poly.area
    point_and_types = area_aware_random_placing(poly, size_list, 0.6, False)
    obj_infos_dict = {}

    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_rest_pavilion(
    terrain_labels: List[List[List[float]]],
    height_map: List[List[float]],
    poly: Polygon,
    pav_datas: List[dict],
    tree_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
    poly_area = poly.area
    obj_infos_list = []
    existing_polys = []
    minx, minz, maxx, maxz = poly.bounds

    # add pavilions
    bminx, bminz, bmaxx, bmaxz = minx + 4, minz + 4, maxx - 4, maxz - 4
    pav_size_list = [(dat["size"][0], dat["size"][2]) for dat in pav_datas]
    num_pavs = min(3, int(poly_area / 300))
    for k in range(num_pavs):
        x, y, z, pav_poly, pav_type = [None for _ in range(5)]
        for att in range(100):
            rand_type = random.randint(0, len(pav_size_list) - 1)
            cx, cz = random.uniform(bminx, bmaxx), random.uniform(bminz, bmaxz)
            xl, xh, zl, zh = (
                cx - pav_size_list[rand_type][0] / 2,
                cx + pav_size_list[rand_type][0] / 2,
                cz - pav_size_list[rand_type][1] / 2,
                cz + pav_size_list[rand_type][1] / 2,
            )
            building_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            if poly.contains(building_poly):
                valid = True
                for ex_poly in existing_polys:
                    if building_poly.distance(ex_poly) < 2:
                        valid = False
                        break
                if valid:
                    heights = [
                        get_height(height_map, xl, zl),
                        get_height(height_map, xl, zh),
                        get_height(height_map, xh, zl),
                        get_height(height_map, xh, zh),
                    ]
                    if max(heights) - min(heights) > 0.01:
                        continue
                    x, y, z, pav_poly, pav_type = cx, sum(heights) / 4, cz, building_poly, rand_type
                    break
        if x is None:
            continue
        #labeling_area(terrain_labels, height_map, pav_poly, 4, 0.2)
        existing_polys.append(pav_poly)
        y *= MAX_HEIGHT
        if pav_datas[pav_type]["name"] == "Pavillion_Medium":
            y -= 1
        obj_infos_list.append(([(x, y, z, random.uniform(0, 2 * pi))], pav_datas[pav_type]["name"]))

    # add trees
    tree_area = deepcopy(poly)
    for ex_poly in existing_polys:
        tree_area = tree_area.difference(ex_poly)
    tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    point_and_types = random_placing(tree_area, tree_size_list, 0.6, True)
    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
        obj_infos_list.append(([(x, y, z, random.uniform(0, 2 * pi))], tree_datas[typ]["name"]))

    return obj_infos_list

def add_pavilion_gv(
    terrain_labels: List[List[List[float]]],
    height_map: List[List[float]],
    poly: Polygon,
    pav_datas: List[dict],
    tree_datas: List[dict],
    plant_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
    poly_area = poly.area
    now_area = 0
    obj_infos_list = []
    existing_polys = []
    minx, minz, maxx, maxz = poly.bounds

    # add pavilions
    bminx, bminz, bmaxx, bmaxz = minx + 4, minz + 4, maxx - 4, maxz - 4
    pav_size_list = [(dat["size"][0], dat["size"][2]) for dat in pav_datas]
    num_pavs = min(3, int(poly_area / 300))
    for k in range(num_pavs):
        x, y, z, pav_poly, pav_type = [None for _ in range(5)]
        for att in range(100):
            rand_type = random.randint(0, len(pav_size_list) - 1)
            cx, cz = random.uniform(bminx, bmaxx), random.uniform(bminz, bmaxz)
            xl, xh, zl, zh = (
                cx - pav_size_list[rand_type][0] / 2,
                cx + pav_size_list[rand_type][0] / 2,
                cz - pav_size_list[rand_type][1] / 2,
                cz + pav_size_list[rand_type][1] / 2,
            )
            building_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            if poly.contains(building_poly):
                valid = True
                for ex_poly in existing_polys:
                    if building_poly.distance(ex_poly) < 2:
                        valid = False
                        break
                if valid:
                    heights = [
                        get_height(height_map, xl, zl),
                        get_height(height_map, xl, zh),
                        get_height(height_map, xh, zl),
                        get_height(height_map, xh, zh),
                    ]
                    if max(heights) - min(heights) > 0.01:
                        continue
                    x, y, z, pav_poly, pav_type = cx, sum(heights) / 4, cz, building_poly, rand_type
                    break
        if x is None:
            continue
        now_area += pav_poly.area
        #labeling_area(terrain_labels, height_map, pav_poly, 4, 0.2)
        existing_polys.append(pav_poly)
        y *= MAX_HEIGHT
        obj_infos_list.append(([(x, y, z, random.uniform(0, 2 * pi))], pav_datas[pav_type]["name"]))

    # add trees
    tree_area = deepcopy(poly)
    for ex_poly in existing_polys:
        tree_area = tree_area.difference(ex_poly)
    tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    point_and_types = random_placing(tree_area, tree_size_list, 0.6, True)
    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
        obj_infos_list.append(([(x, y, z, random.uniform(0, 2 * pi))], tree_datas[typ]["name"]))


    # add trees, HACK, why does the tree not exclude the exisiting_poly?
    #tree_area = poly.buffer(-22) # shrink 22 units, it seems not a proper process
    # if tree_area.area > 0.1:
    #     tree_area = tree_area.simplify(0, False)
    #     tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    #     point_and_types = random_placing(tree_area, tree_size_list, 0.6, True)
    #     obj_infos_dict = {}
    #     for typ_idx in range(len(tree_datas)):
    #         obj_infos_dict[typ_idx] = []

    #     for point_and_type in point_and_types:
    #         x, z, typ = point_and_type
    #         y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
    #         obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    #     for typ_idx in obj_infos_dict:
    #         obj_infos_list.append((obj_infos_dict[typ_idx], tree_datas[typ_idx]["name"]))

    ##################### add tree
    tree_autum_winter_list = ["Acer_buergerianum_A", 
        "Camellia_combination_B", 
        "Camellia_combination_C",
        "Maple_combination_B",
        "Maple_combination_C",
        "Wintersweet_rock_combination_A",
        "Wintersweet_rock_combination_B",
    ]
    tree_list = ["Abelia_grandiflora_stone_A",
        "Asiaticapple_combination_A",
        "Camellia_combination_A", 
        "Cinnamomum_combination_A", 
        "Hydrangea_camellia_rock_combination_A",
        "Magnolia_rock_combination_A",
        "Magnolia_rock_combination_B",
        "Osmanthus_flower_combination_A",
        "Osmanthus_trees_combination_A",
        "Osmanthus_tree_combination_B",
        "Paraso_flower_rock_combination_A",
        "Peach_combination_A",
        "Peach_combination_B",
        "Pine_rock_combination_A",
        "Rock_Pine_combination_A",
        "Cinnamomum_maple_combination_A",
        "Sakura_combination_A",
        "Sakura_combination_B",
        "Sakura_rock_combination_A",
    ]
    tree_data_list = []
    for name in tree_list:
        tree_data_list.append(find_data(name, plant_datas))

    size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_data_list]
    poly_area = poly.area

    # get remaining polygon

    def multipolygons_intersects(polygons, target_poly):
        contains = False
        for poly in polygons:
            if poly.intersects(target_poly):
                contains = True
                break
        return contains

    target_area = poly.area * 0.4
    
    type_areas = [size[0] * size[1] for size in size_list]
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    num_types = len(size_list)
    override = False
    continuous_fail = 0
    for att in range(1000):
        typ = random.randint(0, num_types - 1)
        x = random.uniform(minx, maxx)
        z = random.uniform(minz, maxz)
        length, width = size_list[typ][0], size_list[typ][1]
        half_length = length / 2
        half_width = width / 2
        
        # Define corners (counter-clockwise)
        corners = [
            (x - half_length, z - half_width),  # bottom-left
            (x + half_length, z - half_width),  # bottom-right
            (x + half_length, z + half_width),  # top-right
            (x - half_length, z + half_width)   # top-left
        ]
        
        # Create the polygon
        polygon = Polygon(corners)

        if poly.contains(polygon) and not multipolygons_intersects(existing_polys, polygon):
            valid = True
            if not override: # collision
                for px, pz, p_typ in point_and_types:
                    if (
                        abs(px - x) < (size_list[p_typ][0] + size_list[typ][0]) / 2
                        and abs(pz - z) < (size_list[p_typ][1] + size_list[typ][1]) / 2
                    ):
                        valid = False
                        break
            if valid:
                point_and_types.append((x, z, typ))

                existing_polys.append(polygon)
                now_area += type_areas[typ]
                heights = [
                    get_height(height_map, x - half_length, z - half_width),
                    get_height(height_map, x - half_length, z + half_width),
                    get_height(height_map, x + half_length, z + half_width),
                    get_height(height_map, x + half_length, z + half_width),
                ]
                if max(heights) - min(heights) > 0.02:
                    continue

                obj_infos_list.append(
                    (
                        [(x, sum(heights) / 4 * MAX_HEIGHT, z, random.randint(0, 3)*pi/2)],
                        tree_data_list[typ]["name"],
                    )
                )
                continuous_fail = 0
                if now_area >= target_area:
                    break
            else:
                continuous_fail += 1
                if continuous_fail > 50:
                    override = True

    return obj_infos_list

def add_hugetree(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    poly_area = poly.area
    point_and_types = random_placing(poly, size_list, 0.1, True, 5)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_plantbeds(
    terrain_labels: List[List[List[float]]],
    height_map: List[List[float]],
    poly: Polygon,
    flower_datas: List[dict],
    bush_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    flower_size_list, bush_size_list = [(dat["size"][0], dat["size"][2]) for dat in flower_datas], [
        (dat["size"][0], dat["size"][2]) for dat in bush_datas
    ]
    num_flower_types, num_bush_types = len(flower_size_list), len(bush_size_list)
    rx, rz, gx, gz = 7.5, 6, 9.5, 7.5
    minx, miny, maxx, maxy = poly.bounds
    wnum, hnum = int((maxx - minx) / gx), int((maxy - miny) / gz)

    is_flower = [[0 for j in range(hnum)] for i in range(wnum)]
    pattern = random.randint(0, 15)
    for i in range(wnum):
        for j in range(hnum):
            if (
                (i % 2 == 0 and j % 2 == 0 and pattern == 0)
                or (i % 2 == 0 and j % 2 == 1 and pattern == 1)
                or (i % 2 == 1 and j % 2 == 0 and pattern == 2)
                or (i % 2 == 1 and j % 2 == 1 and pattern == 3)
            ):
                is_flower[i][j] = 1
            elif (
                (i % 2 == 0 and pattern == 4)
                or (i % 2 == 1 and pattern == 5)
                or (j % 2 == 0 and pattern == 6)
                or (j % 2 == 1 and pattern == 7)
            ):
                is_flower[i][j] = 1
            elif (
                (i % 4 < 2 and j % 4 < 2 and pattern == 8)
                or (i % 4 < 2 and j % 4 >= 2 and pattern == 9)
                or (i % 4 >= 2 and j % 4 < 2 and pattern == 10)
                or (i % 4 >= 2 and j % 4 >= 2 and pattern == 11)
            ):
                is_flower[i][j] = 1
            elif pattern >= 12 and random.random() < 0.5:
                is_flower[i][j] = 1

    obj_infos_list = []
    for i in range(wnum):
        for j in range(hnum):
            cx, cz = minx + (i + 0.5) * gx, miny + (j + 0.5) * gz
            xl, xh, zl, zh = cx - rx / 2, cx + rx / 2, cz - rz / 2, cz + rz / 2
            bed_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            if poly.contains(bed_poly):
                labeling_area(terrain_labels, height_map, bed_poly, 4, 0.2)
                if is_flower[i][j] == 0:
                    typ = random.randint(0, num_bush_types - 1)
                    point_and_types = grid_random_placing(bed_poly, [bush_size_list[typ]], 2, 2)
                    obj_infos = []
                    for point_and_type in point_and_types:
                        x, z, _ = point_and_type
                        y = get_height(height_map, x, z) * MAX_HEIGHT
                        obj_infos.append((x, y, z, random.uniform(0, 2 * pi)))
                    obj_infos_list.append((obj_infos, bush_datas[typ]["name"]))
                else:
                    typ = random.randint(0, num_flower_types - 1)
                    point_and_types = random_placing(bed_poly, [flower_size_list[typ]], 0.3, False)
                    obj_infos = []
                    for point_and_type in point_and_types:
                        x, z, _ = point_and_type
                        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.05
                        obj_infos.append((x, y, z, random.uniform(0, 2 * pi)))
                    obj_infos_list.append((obj_infos, flower_datas[typ]["name"]))

    return obj_infos_list


def add_treelines(
    terrain_labels: List[List[List[float]]], height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    num_types = len(size_list)
    rx, rz, gx, gz = 2, 2, 6, 6
    minx, miny, maxx, maxy = poly.bounds
    wnum, hnum = int((maxx - minx) / gx), int((maxy - miny) / gz)
    obj_infos_list = []
    for i in range(wnum):
        for j in range(hnum):
            cx, cz = minx + (i + 0.5) * gx, miny + (j + 0.5) * gz
            xl, xh, zl, zh = cx - rx / 2, cx + rx / 2, cz - rz / 2, cz + rz / 2
            bed_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            if poly.contains(bed_poly):
                labeling_area(terrain_labels, height_map, bed_poly, 4, 0.2)
                typ = random.randint(0, num_types - 1)
                y = get_height(height_map, cx, cz) * MAX_HEIGHT - 0.2
                obj_infos_list.append(([(cx, y, cz, random.uniform(0, 2 * pi))], datas[typ]["name"]))

    return obj_infos_list


def add_building(
    height_map: List[List[float]],
    poly: Polygon,
    build_datas: List[dict],
    statue_datas: List[dict],
    tree_datas: List[dict],
    plant_data_gv: List[dict],
    building_type: str,
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
    poly_area = poly.area
    obj_infos_list = []
    existing_polys = []
    minx, minz, maxx, maxz = poly.bounds
    # add main building (Building_A)
    ## deternube number of main buildings based on area size
    num_main_building = 1
    if poly_area > 750:
        num_main_building = 3
    elif poly_area > 300:
        num_main_building = 2

    house_s_list = ["House_S_A", "House_S_B", "House_S_C", "House_S_D", "House_S_E", "House_S_F"] # > 250 
    house_n_list = ["House_N_A", "House_N_B", "House_N_C", "House_N_D", "House_N_E", "House_N_F"] # > 150
    pavilion_list = ["Pavilion_A", "Pavilion_B", "Pavilion_C", "Pavilion_D", "Pavilion_E", "Pavilion_F"] # > 50
    attic_list = ["Attic_A", "Attic_B", "Attic_C", "Attic_D", "Attic_E"] # > 350
    

    # random
    if building_type == "Attic": 
        main_building_data = find_data(random.choice(attic_list), build_datas)
    elif building_type == "House_S":
        main_building_data = find_data(random.choice(house_s_list), build_datas)
    elif building_type == "House_N":
        main_building_data = find_data(random.choice(house_n_list), build_datas)
    else: 
        main_building_data = find_data(random.choice(pavilion_list), build_datas)


    rx, rz = main_building_data["size"][0], main_building_data["size"][1] # HACK buffer? It should be the real size of buildings
    bminx, bminz, bmaxx, bmaxz = minx + rz / 2, minz + rz / 2, maxx - rz / 2, maxz - rz / 2
    if bminx > bmaxx or bminz > bmaxz:
        num_main_building = 0
    for k in range(num_main_building):
        x, y, z, f_idx, build_poly = [None for _ in range(5)]
        for att in range(100):
            # Random position and orientation, float and int
            cx, cz = random.uniform(bminx, bmaxx), random.uniform(bminz, bmaxz) # center
            forw_idx = random.randint(0, 3)  # direction, 0: 0, 1: pi/2, 2: pi, 3: 3pi/2
            xl, xh, zl, zh = None, None, None, None # length = width = x axis direction, h = height = y axis direction
            if forw_idx == 0 or forw_idx == 2:
                xl, xh, zl, zh = cx - rx / 2, cx + rx / 2, cz - rz / 2, cz + rz / 2
            else:
                xl, xh, zl, zh = cx - rz / 2, cx + rz / 2, cz - rx / 2, cz + rx / 2
            building_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            if poly.contains(building_poly):
                valid = True
                for existing_poly in existing_polys:
                    if building_poly.intersects(existing_poly):
                        valid = False
                        break
                if (
                    valid
                ):  # from the front of the building, cast a ray to intersect with the polygon, calculate the distance
                    point, vec = None, None
                    if forw_idx == 0:
                        point, vec = p(cx, zl), p(0, -1)
                    elif forw_idx == 1:
                        point, vec = p(xh, cz), p(1, 0)
                    elif forw_idx == 2:
                        point, vec = p(cx, zh), p(0, 1)
                    else:
                        point, vec = p(xl, cz), p(-1, 0)
                    vec *= 1000
                    ray = LineString([point, point + vec])
                    inter = ray.intersection(poly.exterior)
                    if isinstance(inter, Point):
                        dis = eu_dist(point, (inter.x, inter.y))
                        if dis < 3:
                            heights = [
                                get_height(height_map, xl, zl),
                                get_height(height_map, xl, zh),
                                get_height(height_map, xh, zl),
                                get_height(height_map, xh, zh),
                            ]
                            if max(heights) - min(heights) > 0.02:
                                continue
                            x, y, z, f_idx, build_poly = cx, sum(heights) / 4, cz, forw_idx, building_poly
                            break
        if x is None:
            continue
        forw = f_idx * pi / 2
        building_x, building_z = x + cos(forw + pi / 2) * 2.5, z + sin(forw + pi / 2) * 2.5
        sa_x, sa_z = x + cos(forw - pi / 2) * 5.5 - cos(forw) * 7, z + sin(forw - pi / 2) * 5.5 - sin(forw) * 7
        sb_x, sb_z = x + cos(forw - pi / 2) * 5.5 + cos(forw) * 7, z + sin(forw - pi / 2) * 5.5 + sin(forw) * 7
        obj_infos_list.append(
            (
                [(building_x, y * MAX_HEIGHT, building_z, forw)],
                main_building_data["name"],
            )
        )
        existing_polys.append(build_poly)

    # add trees
    tree_area = poly.buffer(-22) # shrink 22 units, it seems not a proper process
    if tree_area.area > 0.1:
        tree_area = tree_area.simplify(0, False)
        tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
        point_and_types = random_placing(tree_area, tree_size_list, 0.6, True)
        obj_infos_dict = {}
        for typ_idx in range(len(tree_datas)):
            obj_infos_dict[typ_idx] = []

        for point_and_type in point_and_types:
            x, z, typ = point_and_type
            y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
            obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

        for typ_idx in obj_infos_dict:
            obj_infos_list.append((obj_infos_dict[typ_idx], tree_datas[typ_idx]["name"]))

    return obj_infos_list

def add_building_by_optimization(
    height_map: List[List[float]],
    poly: Polygon,
    build_datas: List[dict],
    plant_datas: List[dict],
    rock_datas: List[dict],
    tree_datas: List[dict],
    building_type: str,
    water_poly,
    terrain_labels,
    pavilion_data_gv
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
    poly_area = poly.area
    obj_infos_list = []
    existing_polys = []
    now_area = 0
    minx, minz, maxx, maxz = poly.bounds
    # add main building (Building_A)
    ## deternube number of main buildings based on area size
    num_main_building = 1
    if poly_area > 800:
        num_main_building = 3
    elif poly_area > 500:
        num_main_building = 2
    # main_building_data, stata_data, statb_data = (
    #     find_data("Building_A", build_datas),
    #     find_data("Statue_A", statue_datas),
    #     find_data("Statue_B", statue_datas),
    # )
    # Create a list

    house_s_list = ["House_S_A", "House_S_B", "House_S_C", "House_S_D", "House_S_E", "House_S_F"] # > 250 
    house_n_list = ["House_N_A", "House_N_B", "House_N_C", "House_N_D", "House_N_E", "House_N_F"] # > 150
    pavilion_list = ["Pavilion_A", "Pavilion_B", "Pavilion_C", "Pavilion_D", "Pavilion_E", "Pavilion_F"] # > 50
    attic_list = ["Attic_A", "Attic_B", "Attic_C", "Attic_D", "Attic_E"] # > 350
    
    main_building_data = []
    
    
    if building_type == "Attic": 
        main_building_data = find_data(random.choice(attic_list), build_datas)
    elif building_type == "House_S":
        main_building_data = find_data(random.choice(house_s_list), build_datas)
    elif building_type == "House_N":
        main_building_data = find_data(random.choice(house_n_list), build_datas)
    else: 
        main_building_data = find_data(random.choice(pavilion_list), build_datas)


    rx, rz = main_building_data["size"][0], main_building_data["size"][1] # HACK buffer? It should be the real size of buildings
    bminx, bminz, bmaxx, bmaxz = minx + rz / 2, minz + rz / 2, maxx - rz / 2, maxz - rz / 2
    if bminx > bmaxx or bminz > bmaxz:
        num_main_building = 0
    for k in range(num_main_building):
        x, y, z, f_idx, build_poly = [None for _ in range(5)]
        for att in range(1000): # default 100
            # Random position and orientation, float and int
            cx, cz = random.uniform(bminx, bmaxx), random.uniform(bminz, bmaxz) # center
            forw_idx = random.randint(0, 3)  # direction, 0: 0, 1: pi/2, 2: pi, 3: 3pi/2
            xl, xh, zl, zh = None, None, None, None # length = width = x axis direction, h = height = y axis direction
            if forw_idx == 0 or forw_idx == 2:
                xl, xh, zl, zh = cx - rx / 2, cx + rx / 2, cz - rz / 2, cz + rz / 2
            else:
                xl, xh, zl, zh = cx - rz / 2, cx + rz / 2, cz - rx / 2, cz + rx / 2
            building_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            if poly.contains(building_poly):
                valid = True
                for existing_poly in existing_polys:
                    if building_poly.intersects(existing_poly):
                        valid = False
                        break
                if (
                    valid
                ):  # from the front of the building, cast a ray to intersect with the polygon, calculate the distance
                    point, vec = None, None
                    if forw_idx == 0:
                        point, vec = p(cx, zl), p(0, -1)
                    elif forw_idx == 1:
                        point, vec = p(xh, cz), p(1, 0)
                    elif forw_idx == 2:
                        point, vec = p(cx, zh), p(0, 1)
                    else:
                        point, vec = p(xl, cz), p(-1, 0)
                    vec *= 1000
                    ray = LineString([point, point + vec])
                    inter = ray.intersection(poly.exterior)
                    if isinstance(inter, Point):
                        dis = eu_dist(point, (inter.x, inter.y))
                        if dis < 3:
                            heights = [
                                get_height(height_map, xl, zl),
                                get_height(height_map, xl, zh),
                                get_height(height_map, xh, zl),
                                get_height(height_map, xh, zh),
                            ]
                            if max(heights) - min(heights) > 0.02:
                                continue
                            x, y, z, f_idx, build_poly = cx, sum(heights) / 4, cz, forw_idx, building_poly
                            break       
        if x is None:
            continue
        now_area += build_poly.area
        forw = f_idx * pi / 2
        building_x, building_z = x + cos(forw + pi / 2) * 2.5, z + sin(forw + pi / 2) * 2.5
        sa_x, sa_z = x + cos(forw - pi / 2) * 5.5 - cos(forw) * 7, z + sin(forw - pi / 2) * 5.5 - sin(forw) * 7
        sb_x, sb_z = x + cos(forw - pi / 2) * 5.5 + cos(forw) * 7, z + sin(forw - pi / 2) * 5.5 + sin(forw) * 7
        obj_infos_list.append(
            (
                [(building_x, y * MAX_HEIGHT, building_z, forw)],
                main_building_data["name"],
            )
        )
        # obj_infos_list.append(
        #     (
        #         [(sa_x, get_height(height_map, sa_x, sa_z) * MAX_HEIGHT, sa_z, forw + pi)],
        #         stata_data["name"],
        #     )
        # )
        # obj_infos_list.append(
        #     (
        #         [(sb_x, get_height(height_map, sb_x, sb_z) * MAX_HEIGHT, sb_z, forw + pi)],
        #         statb_data["name"],
        #     )
        # )
        existing_polys.append(build_poly)

    # add view pavilion
    if poly.distance(water_poly) < 1: #poly.touches(water_poly) : # if the area is connected to water area, set the view pavillion
        #pavillion_view_data = find_data("Pavillion_B", pavilion_data)
        allocated_union = unary_union(existing_polys)
        remaining_polys = poly.difference(allocated_union)
        if isinstance(remaining_polys, MultiPolygon): # get the polygon with most space
            #remaining_poly = unary_union(remaining_polys)
            remaining_poly = None
            max_area = -1
            for mp in remaining_polys:
                current_area = mp.area
                if current_area > max_area:
                    max_area = current_area
                    remaining_poly = mp

        else: remaining_poly = remaining_polys
        obj_infos = add_view_pavilion(terrain_labels, height_map, remaining_poly, water_poly, pavilion_data_gv)


    # add trees, HACK, why does the tree not exclude the exisiting_poly?
    #tree_area = poly.buffer(-22) # shrink 22 units, it seems not a proper process
    # if tree_area.area > 0.1:
    #     tree_area = tree_area.simplify(0, False)
    #     tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    #     point_and_types = random_placing(tree_area, tree_size_list, 0.6, True)
    #     obj_infos_dict = {}
    #     for typ_idx in range(len(tree_datas)):
    #         obj_infos_dict[typ_idx] = []

    #     for point_and_type in point_and_types:
    #         x, z, typ = point_and_type
    #         y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
    #         obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    #     for typ_idx in obj_infos_dict:
    #         obj_infos_list.append((obj_infos_dict[typ_idx], tree_datas[typ_idx]["name"]))

    ##################### add tree
    tree_autum_winter_list = ["Acer_buergerianum_A", 
        "Camellia_combination_B", 
        "Camellia_combination_C",
        "Maple_combination_B",
        "Maple_combination_C",
        "Wintersweet_rock_combination_A",
        "Wintersweet_rock_combination_B",
    ]
    tree_list = ["Abelia_grandiflora_stone_A",
        "Asiaticapple_combination_A",
        "Camellia_combination_A", 
        "Cinnamomum_combination_A", 
        "Hydrangea_camellia_rock_combination_A",
        "Magnolia_rock_combination_A",
        "Magnolia_rock_combination_B",
        "Osmanthus_flower_combination_A",
        "Osmanthus_trees_combination_A",
        "Osmanthus_tree_combination_B",
        "Paraso_flower_rock_combination_A",
        "Peach_combination_A",
        "Peach_combination_B",
        "Pine_rock_combination_A",
        "Rock_Pine_combination_A",
        "Cinnamomum_maple_combination_A",
        "Sakura_combination_A",
        "Sakura_combination_B",
        "Sakura_rock_combination_A",
    ]
    tree_data_list = []
    for name in tree_list:
        tree_data_list.append(find_data(name, plant_datas))

    size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_data_list]
    poly_area = poly.area

    # get remaining polygon

    def multipolygons_intersects(polygons, target_poly):
        contains = False
        for poly in polygons:
            if poly.intersects(target_poly):
                contains = True
                break
        return contains

    target_area = poly.area * 0.4
    
    type_areas = [size[0] * size[1] for size in size_list]
    point_and_types = []
    minx, minz, maxx, maxz = poly.bounds
    num_types = len(size_list)
    override = False
    continuous_fail = 0
    for att in range(1000):
        typ = random.randint(0, num_types - 1)
        x = random.uniform(minx, maxx)
        z = random.uniform(minz, maxz)
        length, width = size_list[typ][0], size_list[typ][1]
        half_length = length / 2
        half_width = width / 2
        
        # Define corners (counter-clockwise)
        corners = [
            (x - half_length, z - half_width),  # bottom-left
            (x + half_length, z - half_width),  # bottom-right
            (x + half_length, z + half_width),  # top-right
            (x - half_length, z + half_width)   # top-left
        ]
        
        # Create the polygon
        polygon = Polygon(corners)

        if poly.contains(polygon) and not multipolygons_intersects(existing_polys, polygon):
            valid = True
            if not override: # collision
                for px, pz, p_typ in point_and_types:
                    if (
                        abs(px - x) < (size_list[p_typ][0] + size_list[typ][0]) / 2
                        and abs(pz - z) < (size_list[p_typ][1] + size_list[typ][1]) / 2
                    ):
                        valid = False
                        break
            if valid:
                point_and_types.append((x, z, typ))

                existing_polys.append(polygon)
                now_area += type_areas[typ]
                heights = [
                    get_height(height_map, x - half_length, z - half_width),
                    get_height(height_map, x - half_length, z + half_width),
                    get_height(height_map, x + half_length, z + half_width),
                    get_height(height_map, x + half_length, z + half_width),
                ]
                if max(heights) - min(heights) > 0.02:
                    continue

                obj_infos_list.append(
                    (
                        [(x, sum(heights) / 4 * MAX_HEIGHT, z, random.randint(0, 3)*pi/2)],
                        tree_data_list[typ]["name"],
                    )
                )
                continuous_fail = 0
                if now_area >= target_area:
                    break
            else:
                continuous_fail += 1
                if continuous_fail > 50:
                    override = True

    return obj_infos_list, existing_polys, now_area

def add_view_pavilion(
    terrain_labels: List[List[List[float]]],
    height_map: List[List[float]],
    poly: Polygon,
    water_poly: Polygon,
    pav_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
    poly_area = poly.area
    obj_infos_list = []
    existing_polys = []
    minx, minz, maxx, maxz = poly.bounds # bbox

    # find best keypoint
    def is_visible(origin, target, polygon):
        ray = LineString([origin, target])
        inter = ray.intersection(polygon)
        # If the intersection is empty, the ray doesn't go through the polygon at all
        if inter.is_empty:
            return False
        # Compare lengths: visible if the intersected part fully matches the ray, if intersection is point, it also return 0
        return inter.length >= ray.length - 1e-8  # small tolerance for floating-point errors

    def compute_visibility_polygon(origin, polygon):
        vertices = [Point(x, y) for x, y in polygon.exterior.coords] # list(polygon.exterior.coords)
        visible_points = []
        for vertex in vertices:
            if vertex == origin:
                continue
            #point = Point(vertex[0], vertex[1])
            if is_visible(origin, vertex, polygon):
                visible_points.append(vertex)

        # Sort by angle around the origin
        visible_points.sort(key=lambda p: np.arctan2(p.y - origin.y, p.x - origin.x))
        coords = [origin.coords[0]] + [p.coords[0] for p in visible_points]
        return Polygon(coords) if len(coords) >= 3 else origin

    def find_best_edge_viewpoint(polygon, samples_per_edge=10):
        #candidates = sample_edge_points(polygon, samples_per_edge)
        #candidates = list(polygon.exterior.coords)
        candidates = [Point(x, y) for x, y in polygon.exterior.coords]
        best_point = None
        max_visible = -1
        best_vis_poly = None
        for pt in candidates:
            vis_poly = compute_visibility_polygon(pt, polygon)
            visible_count = len(vis_poly.exterior.coords) - 1 if vis_poly.geom_type == 'Polygon' else 0
            if visible_count > max_visible:
                max_visible = visible_count
                best_point = pt
                best_vis_poly = vis_poly
        return best_point, best_vis_poly

    best_position = find_best_edge_viewpoint(water_poly)
    # add pavilions
    bminx, bminz, bmaxx, bmaxz = minx + 2.7, minz + 2.7, maxx - 2.7, maxz - 2.7
    pav_size_list = [(dat["size"][0], dat["size"][2]) for dat in pav_datas]
    num_pavs = 1 #min(3, int(poly_area / 300))
    for k in range(num_pavs):
        x, y, z, pav_poly, pav_type = [None for _ in range(5)]
        for att in range(100):
            rand_type = 2 #random.randint(0, len(pav_size_list) - 1)
            cx, cz = random.uniform(bminx, bmaxx), random.uniform(bminz, bmaxz)
            xl, xh, zl, zh = (
                cx - pav_size_list[rand_type][0] / 2,
                cx + pav_size_list[rand_type][0] / 2,
                cz - pav_size_list[rand_type][1] / 2,
                cz + pav_size_list[rand_type][1] / 2,
            )
            building_poly = Polygon([(xl, zl), (xl, zh), (xh, zh), (xh, zl)])
            current_center = Point(cx, cz)
            if poly.contains(building_poly):# and current_center.distance(best_position) < 5:
                valid = True
                for ex_poly in existing_polys:
                    if building_poly.distance(ex_poly) < 2:
                        valid = False
                        break
                if valid:
                    heights = [
                        get_height(height_map, xl, zl),
                        get_height(height_map, xl, zh),
                        get_height(height_map, xh, zl),
                        get_height(height_map, xh, zh),
                    ]
                    if max(heights) - min(heights) > 0.01:
                        continue
                    x, y, z, pav_poly, pav_type = cx, sum(heights) / 4, cz, building_poly, rand_type
                    print("best position: {best_position.x} , {best_position.y}")
                    break
        if x is None:
            continue
        #labeling_area(terrain_labels, height_map, pav_poly, 4, 0.2)
        existing_polys.append(pav_poly)
        y *= MAX_HEIGHT
        if pav_datas[pav_type]["name"] == "Pavillion_Medium":
            y -= 1
        obj_infos_list.append(([(x, y, z, 0)], pav_datas[pav_type]["name"]))

    # add trees
    # tree_area = deepcopy(poly)
    # for ex_poly in existing_polys:
    #     tree_area = tree_area.difference(ex_poly)
    # tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    # point_and_types = random_placing(tree_area, tree_size_list, 1, True)
    # for point_and_type in point_and_types:
    #     x, z, typ = point_and_type
    #     y = get_height(height_map, x, z) * MAX_HEIGHT - 0.2
    #     obj_infos_list.append(([(x, y, z, random.uniform(0, 2 * pi))], tree_datas[typ]["name"]))

    return obj_infos_list

def add_hillrock(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    point_and_types = group_random_placing(poly, size_list, 0.01, 2, 2, 3)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - 0.1
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_plants(
    height_map: List[List[float]], poly: Polygon, plant_datas: List[dict], bush_datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    plant_size_list, bush_size_list = [(dat["size"][0], dat["size"][2]) for dat in plant_datas], [
        (dat["size"][0], dat["size"][2]) for dat in bush_datas
    ]
    plant_point_and_types = random_placing(poly, plant_size_list, 0.3, True)
    bush_point_and_types = random_placing(poly, bush_size_list, 0.1, True)
    plant_obj_infos_dict, bush_obj_infos_dict = {}, {}
    for typ_idx in range(len(plant_datas)):
        plant_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(bush_datas)):
        bush_obj_infos_dict[typ_idx] = []

    for point_and_type in plant_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        plant_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in bush_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        bush_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in plant_obj_infos_dict:
        obj_infos_list.append((plant_obj_infos_dict[typ_idx], plant_datas[typ_idx]["name"]))
    for typ_idx in bush_obj_infos_dict:
        obj_infos_list.append((bush_obj_infos_dict[typ_idx], bush_datas[typ_idx]["name"]))
    return obj_infos_list


def labeling_area(
    terrain_labels: List[List[List[float]]], height_map: List[List[float]], poly: Polygon, label: int, buffer: float
) -> None:
    """
    input: 3D numpy array resolution_grid,
    description: label the grid, and adjust height map

    """
    points = [xy2idx(x, y) for x, y in poly.exterior.coords]
    idx_poly = Polygon(points)
    # decide what units are covered by the idx_poly
    minx, miny, maxx, maxy = idx_poly.bounds
    ratio = max(MAP_W, MAP_H) / (RL * max(W, H)) # used to transfer layout size to normal real scene

    x1, z1 = int(minx - buffer * ratio), int(miny - buffer * ratio) # y corresponds to z in Unity
    x2, z2 = int(maxx + buffer * ratio) + 1, int(maxy + buffer * ratio) + 1 # 
    # set terrain labels in and around polygon, 
    # HACK how to make it more efficient?
    for i in range(x1, x2):
        for j in range(z1, z2):
            if i < 0 or i >= MAP_W or j < 0 or j >= MAP_H:
                continue
            pcenter = Point(i + 0.5, j + 0.5)
            if idx_poly.contains(pcenter):
                terrain_labels[i][j][label] += 1
                if label == 1: # main road
                    height_map[i][j] += 0.15 / MAX_HEIGHT
                elif label == 2: # secondary road
                    height_map[i][j] += 0.1 / MAX_HEIGHT
            else: # labeling according to the distance
                dis = pcenter.distance(idx_poly) / ratio
                if dis < buffer:
                    rat = 1 - dis / buffer
                    terrain_labels[i][j][label] += rat
                    if label == 1:
                        height_map[i][j] += 0.15 / MAX_HEIGHT * rat
                    elif label == 2:
                        height_map[i][j] += 0.1 / MAX_HEIGHT * rat


def labeling_height_map(
    terrain_labels: List[List[List[float]]], height_map: List[List[float]], poly: Polygon, label: int, buffer: float
) -> None:
    """
    input: 3D numpy array resolution_grid,
    description: label the grid, and adjust height map

    """
    points = [xy2idx(x, y) for x, y in poly.exterior.coords]
    idx_poly = Polygon(points)
    # decide what units are covered by the idx_poly
    minx, miny, maxx, maxy = idx_poly.bounds
    ratio = max(MAP_W, MAP_H) / (RL * max(W, H)) # used to transfer layout size to normal real scene

    x1, z1 = int(minx - buffer * ratio), int(miny - buffer * ratio) # y corresponds to z in Unity
    x2, z2 = int(maxx + buffer * ratio) + 1, int(maxy + buffer * ratio) + 1 # 
    # set terrain labels in and around polygon, 
    # HACK how to make it more efficient?
    for i in range(x1, x2):
        for j in range(z1, z2):
            if i < 0 or i >= MAP_W or j < 0 or j >= MAP_H:
                continue
            pcenter = Point(i + 0.5, j + 0.5)
            if idx_poly.contains(pcenter):
                terrain_labels[i][j][label] += 1
                if label == 1: # main road
                    height_map[i][j] += 0.15 / MAX_HEIGHT
                elif label == 2: # secondary road
                    height_map[i][j] += 0.1 / MAX_HEIGHT
            else: # labeling according to the distance
                dis = pcenter.distance(idx_poly) / ratio
                if dis < buffer:
                    rat = 1 - dis / buffer
                    terrain_labels[i][j][label] += rat
                    if label == 1:
                        height_map[i][j] += 0.15 / MAX_HEIGHT * rat
                    elif label == 2:
                        height_map[i][j] += 0.1 / MAX_HEIGHT * rat


def decide_view_points(
    height_map: List[List[float]], keyunits: List[Tuple[int, int]]
) -> List[Tuple[float, float, float, float, float]]:
    view_points = []
    # based on keyunits
    # for keyunit in keyunits:
    #     x_idx, z_idx = keyunit
    #     xl, xh, zl, zh = x_idx * RL, (x_idx + 1) * RL, z_idx * RL, (z_idx + 1) * RL
    #     for i in range(4):
    #         x, z = random.uniform(xl, xh), random.uniform(zl, zh)
    #         y = get_height(height_map, x, z) * MAX_HEIGHT + 10
    #         yrot = i * pi / 2 + pi / 4
    #         xrot = 20 * pi / 180
    #         view_points.append((x, y, z, xrot, yrot))

    # specified
    x1, x2, x3, x4, x5 = 0.05 * W * RL, 0.15 * W * RL, 0.5 * W * RL, 0.85 * W * RL, 0.95 * W * RL
    z1, z2, z3, z4, z5 = 0.05 * H * RL, 0.15 * H * RL, 0.5 * H * RL, 0.85 * H * RL, 0.95 * H * RL
    # four corners
    xrot = 25 * pi / 180
    view_points.append((x1, 0, z1, xrot, pi / 4))
    view_points.append((x1, 0, z5, xrot, -pi / 4))
    view_points.append((x5, 0, z1, xrot, 3 * pi / 4))
    view_points.append((x5, 0, z5, xrot, -3 * pi / 4))
    # four edges
    view_points.append((x2, 0, z3, xrot, pi / 4))
    view_points.append((x2, 0, z3, xrot, -pi / 4))
    view_points.append((x3, 0, z2, xrot, 3 * pi / 4))
    view_points.append((x3, 0, z2, xrot, pi / 4))
    view_points.append((x4, 0, z3, xrot, -3 * pi / 4))
    view_points.append((x4, 0, z3, xrot, 3 * pi / 4))
    view_points.append((x3, 0, z4, xrot, -pi / 4))
    view_points.append((x3, 0, z4, xrot, -3 * pi / 4))
    # central
    for i in range(4):
        view_points.append((x3, 0, z3, xrot, i * pi / 2))

    for i in range(len(view_points)):
        x, y, z, xrot, yrot = view_points[i]
        y = get_height(height_map, x, z) * MAX_HEIGHT + 15
        yrot = pi / 2 - yrot
        view_points[i] = (x, y, z, xrot, yrot)

    return view_points


def pcg(
    args,
    height_map: List[List[float]],
    points: List[Tuple[float, float]],
    edges: List[Tuple[List[Tuple[int, int]], List[int]]],
    areas: List[Tuple[int, List[int]]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[int, int]], dict],
    parameters: dict,
    data: dict,
    garden_verse: dict,
    gen_idx: int,
    suffix: str = "",
    text: str = "",
    use_selector: bool = True,
    use_solver: bool = True
) -> None:
    """
    points[i]: (x, y), W*RL X H*RL
    edges[i]: (points, info) info: 0: wall, 1: main road, 2: secondary road, 3: type border, 4: lakeside, 5: bridge, 6: wall with hole
    edges[i][0][j]: (point_idx1, point_idx2)
    areas[i]: (type, circuit) type: 1: water, 2: lotus, 3: lake rock, 4: pure land, 5: grass/flower, 6: bush, 7: tree,
                                    8: pavilion, 9: plaza, 10: flowerbed, 11: tree lines, 12: building, 13: hill rock, 14: grass
    areas[i][1][j]: point_idx
    """
    visualize = False
    points = [(x * RL, y * RL) for x, y in points] # WxH to [W*RL, H*RL]
    if visualize:
        # Load scene objects from JSON and visualize them
        scene_path = r"e:\Scientific Research\conlan\scene_0.json"

        def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
            return atan2(2.0 * (qw * qy + qz * qx), 1.0 - 2.0 * (qy * qy + qz * qz))

        with open(scene_path, "r", encoding="utf-8") as f:
            scene = json.load(f)

        name_to_infos = {}
        for node in scene.get("tree", []):
            obj_name = node.get("name")
            transforms = node.get("transforms", [])
            if not obj_name or not transforms:
                continue

            infos = name_to_infos.setdefault(obj_name, [])
            for t in transforms:
                pos = t.get("position", {})
                rot = t.get("rotation", {})
                x = float(pos.get("x", 0.0))
                y = float(pos.get("y", 0.0))
                z = float(pos.get("z", 0.0))
                qx = float(rot.get("x", 0.0))
                qy = float(rot.get("y", 0.0))
                qz = float(rot.get("z", 0.0))
                qw = float(rot.get("w", 1.0))
                yaw = quaternion_to_yaw(qx, qy, qz, qw)
                infos.append((x, y, z, yaw))

        all_obj_infos_from_json = [(infos, name) for name, infos in name_to_infos.items() if infos]

        output_visualize(all_obj_infos_from_json, data, garden_verse, points, edges, areas, infrastructure, "structure")
        return
    
    logpath = "logs/" + str(gen_idx) + suffix + ".txt"
    sys.stdout = open(logpath, "a")
    sys.stderr = open(logpath, "a")
    print("PCG Begins")
    entrance_points, keyunits, edge_used = infrastructure
    entrance_points = [(x * RL, y * RL) for x, y in entrance_points]
    points = [(x * RL, y * RL) for x, y in points] # WxH to [W*RL, H*RL]
    (
        lotus_data,
        lotusgroup_data,
        plant_data,
        lake_rock_data,
        flower_data,
        plantbush_data,
        bush_data,
        bigbush_data,
        rectbush_data,
        bamboo_data,
        hugetree_data,
        bigtree_data,
        pavilion_data,
        building_data,
        hill_rock_data,
        waterside_rock_data,
        wall_data,
        bridge_data,
        corridor_data,
        statue_data,
    ) = (
        data["lotus"],
        data["lotus_group"],
        data["plant"],
        data["lake_rock"],
        data["flower"],
        data["plantbush"],
        data["bush"],
        data["bigbush"],
        data["rectbush"],
        data["bamboo"],
        data["hugetree"],
        data["bigtree"],
        data["pavilion"],
        data["building"],
        data["hill_rock"],
        data["waterside_rock"],
        data["wall"],
        data["bridge"],
        data["corridor"],
        data["statue"],
    )

    (
        wall_data_gv,
        house_data_gv,
        pavilion_data_gv,
        attic_data_gv,
        bridge_data_gv,
        corridor_data_gv,
        plant_data_gv,
        rock_data_gv,
    ) = (
        garden_verse["Wall"],
        garden_verse["House"],
        garden_verse["Pavilion"],
        garden_verse["Attic"],
        garden_verse["Bridge"],
        garden_verse["Corridor"],
        garden_verse["plant"],
        garden_verse["rock"],
    )



    constraints = {"direction relation": ""}

    terrain_labels = np.zeros((MAP_W, MAP_H, 5))  # 3d array, trd: 0: none, 1: main road, 2: secondary road, 3: plaza for human, 4: land
    all_obj_infos = []

    edge2info = {}
    for edge_group in edges:
        edge_idxs, info = edge_group
        for i in range(len(edge_idxs)):
            edge2info[edge_idxs[i]] = info
            edge2info[(edge_idxs[i][1], edge_idxs[i][0])] = info # robust for bidirectional edge indices, convenient for 

    print("handle edges:")
    for edge_group in tqdm(edges):
        edge_idxs, info = edge_group
        for i in range(len(edge_idxs) - 1):
            if edge_idxs[i][1] != edge_idxs[i + 1][0]:
                raise ValueError("Edge not connected")
        point_idxs = [idx[0] for idx in edge_idxs] + [edge_idxs[-1][1]] # collect all points' indices
        point_locs = [points[idx] for idx in point_idxs]
        rand_num = random.random()
        total_length = 0
        for i in range(len(point_locs) - 1):
            total_length += eu_dist(point_locs[i], point_locs[i + 1])
        if 3 in info:  # type border
            pass
        elif 5 in info:  # bridge
            longbridge_data, zigzagbridge_data = find_data("Bridge", bridge_data), find_data(
                "Zigzag_Bridge", bridge_data
            )
            # default, the secondary road should assign zigzag bridge
            if (2 in info) and rand_num < 0.7:
                obj_infos = build_zigzagbridge(height_map, point_locs, zigzagbridge_data)
                all_obj_infos.append((obj_infos, "Zigzag_Bridge"))
            else:
                obj_infos, road_idxs = build_bridge(height_map, point_locs, longbridge_data)
                if obj_infos == -1 and road_idxs == -1: # if the long bridge can not be placed
                    obj_infos = build_zigzagbridge(height_map, point_locs, zigzagbridge_data)
                    all_obj_infos.append((obj_infos, "Zigzag_Bridge"))
                else:
                    all_obj_infos.append((obj_infos, "Bridge"))
                    for idx in road_idxs:
                        terrain_labels[idx[0]][idx[1]][1] += 1
        elif 6 in info:  # entrance
            wall4_data = find_data("Wall_400x300", wall_data)
            obj_infos = build_entrance(height_map, point_locs, wall4_data, entrance_points)
            all_obj_infos.append((obj_infos, "Wall_400x300"))
        elif 0 in info:  # wall
            wall4_data = find_data("Wall_400x300", wall_data)
            obj_infos = build_wall(height_map, point_locs, wall4_data)
            all_obj_infos.append((obj_infos, "Wall_400x300"))
        elif (1 in info) or (2 in info and 4 not in info):  # corridor road
            adding_corridor = True
            if adding_corridor:
                short_corridor_data = find_data("Short_Corridor", corridor_data)
                obj_infos = build_shortCorridor(height_map, point_locs, short_corridor_data)
                all_obj_infos.append((obj_infos, "Short_Corridor"))

            else:
                poly = None
                for i in range(len(point_locs) - 1):
                    # questioning this caculating method
                    sp, ep = p(point_locs[i]), p(point_locs[i + 1])
                    tang = p(ep[0] - sp[0], ep[1] - sp[1])
                    tang = tang / np.linalg.norm(tang)
                    norm = p(ep[1] - sp[1], sp[0] - ep[0])
                    norm = norm / np.linalg.norm(norm)
                    road_width = MAIN_ROAD_WIDTH if (1 in info) else SUB_ROAD_WIDTH
                    new_poly = Polygon(
                        [
                            sp + norm * road_width / 2 - tang * 1e-3,
                            sp - norm * road_width / 2 - tang * 1e-3,
                            ep - norm * road_width / 2 + tang * 1e-3,
                            ep + norm * road_width / 2 + tang * 1e-3,
                        ]
                    )
                    if poly is None:
                        poly = new_poly
                    else:
                        poly = poly.union(new_poly) # combien polygons
                poly = poly.simplify(0, False)  # strongly simplify the polygon
                #labeling_area(terrain_labels, height_map, poly, 1 if (1 in info) else 2, 1)
        elif (2 in info) and (4 in info):  # lakeside
            #w_rock_data = find_data("Waterside_Rock_A", waterside_rock_data)
            obj_infos = build_lakeside_rock(height_map, point_locs, waterside_rock_data, info)
            all_obj_infos += obj_infos
            # all_obj_infos.append((obj_infos, "Waterside_Rock_A"))
        
    lakeside_buffer = 3
    bridge_buffer = 3
    wall_buffer = 0.5
    
    print("handle areas:")
    ###### cacualte necessary info
    # get the whole water area, firstly considering the single water area
    water_att = [1,2,3]
    water_circuit = []
    water_poly = Polygon()
    area_info = []

    # labeling
    for area in areas:
        area_type, circuit = area
        poly = Polygon([points[idx] for idx in circuit])
        if area_type in [4,5,6,7,8]:
            labeling_area(terrain_labels, height_map, poly, 4, 1)  
        if area_type in [9,10,11,12]:
            labeling_area(terrain_labels, height_map, poly, 3, 0.5) 
        #labeling_height_map(terrain_labels, height_map, poly, 4, 1)
    
    output_label_map(terrain_labels, MAP_W, MAP_H, "label_map_" + str(gen_idx), suffix)
    output_height_map(height_map, MAP_W, MAP_H, "height_map_" + str(gen_idx), suffix)

    #return
    for area in areas:
        area_type, circuit = area
        # get info from edge info

        # get each area
        if len(circuit) < 3:
            # for i in range(1000):
            #     print("-----------------")
            # print(circuit)
            # for edge_group in edges:
            #     edge_idxs, info = edge_group
            #     print(edge_idxs)
            continue
        poly = Polygon([points[idx] for idx in circuit])
        # NOTE remove the boundary buffer area
        buffer = None
        area_info_numb = None # (1) 0 is the water area; (2) 1 is land adjacent to water; (3) 2 is land adjacent to garden boundary; (4) 3 is land not connected to water or boundary,
        for i in range(len(circuit)):
            idx1, idx2 = circuit[i], circuit[(i + 1) % len(circuit)]
            if idx1 == idx2:
                continue
            p1, p2 = p(points[idx1]), p(points[idx2])
            if (idx1, idx2) in edge2info:
                edge_info = edge2info[(idx1, idx2)]
                if 3 in edge_info:  
                    area_info_numb = 2    # boundary
                    continue
                if 5 in edge_info:  buffer = bridge_buffer
                if 0 in edge_info:  buffer = wall_buffer
                if 4 in edge_info:  buffer = lakeside_buffer
                if 1 in edge_info:  buffer = MAIN_ROAD_WIDTH / 2    # 2
                if 2 in edge_info or 6 in edge_info:  buffer = SUB_ROAD_WIDTH         # 2
                if 7 in edge_info or 8 in edge_info or 9 in edge_info or 10 in edge_info: area_info_numb = 1 # adjacent to water
            else:
                continue
            tang = p2 - p1
            tang = tang / np.linalg.norm(tang)
            norm = p(p2[1] - p1[1], p1[0] - p2[0])
            norm = norm / np.linalg.norm(norm)
            if (tang is None) or (norm is None):
                continue
            diff_poly = Polygon(
                [
                    p1 + norm * buffer - tang * 0.5,
                    p1 - norm * buffer - tang * 0.5,
                    p2 - norm * buffer + tang * 0.5,
                    p2 + norm * buffer + tang * 0.5,
                ]
            )
            poly = poly.difference(diff_poly) # remove the buffer area
        if area_type in [1,2,3]: 
            area_info_numb = 0  # water area, water area is prior
        elif area_info_numb == None:
            area_info_numb = 3  # land not connected to water or boundary
        
        poly = poly.simplify(0, False) # strongly simplify
        polys = [poly] if isinstance(poly, Polygon) else list(poly.geoms) # single polygon or multiple polygons
        
        # get necessray info for selector and solver
        area_info.append((area_info_numb, sum(p.area for p in polys)))

        # process the water area
        if area_type in water_att:
            poly = Polygon([points[idx] for idx in circuit])
            water_poly = water_poly.union(poly) # TODO, the water poly has multiple connected areas
            #water_circuit.extend(circuit)
        if isinstance(water_poly, MultiPolygon):
            water_poly = unary_union(water_poly)
            if isinstance(water_poly, MultiPolygon):
                # If still a MultiPolygon after union, take the largest polygon
                largest = max(water_poly.geoms, key=lambda x: x.area)
                water_poly = largest

    #use_selector = False
    use_selector = False
    if args.use_conlan: use_selector = False
    elif args.use_conlan_layout: use_selector = False

    if use_selector:
        object_selection = {}
        selector_prompts = object_selection_prompt.replace("*{area}*", str(area_info)).replace("*{area_num}*", str(len(area_info))).replace("*{text}*", text)
        selector_response = asset_selector(selector_prompts)
        object_selection = parse_asset_selector_gptres(selector_response)
    else:
        object_selection ={
            "area_0": {
                "Waterside_rock_combination_E": [
                    "Rock",
                    1
                ]
            },
            "area_1": {
                "Waterside_rock_combination_A": [
                    "Rock",
                    1
                ]
            },
            "area_2": {
                "Waterside_rock_combination_B": [
                    "Rock",
                    1
                ]
            },
            "area_3": {},
            "area_4": {
                "House_S_A": [
                    "House",
                    1
                ],
                "Pavilion_B": [
                    "Pavilion",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ],
                "Bamboo_rock_combination_A": [
                    "Plant",
                    3
                ]
            },
            "area_5": {
                "House_N_C": [
                    "House",
                    1
                ],
                "Pavilion_A": [
                    "Pavilion",
                    1
                ],
                "Rock_musa_combination_A": [
                    "Rock",
                    2
                ],
                "Bamboo_rock_combination_C": [
                    "Plant",
                    1
                ]
            },
            "area_6": {
                "Waterside_rock_combination_E": [
                    "Rock",
                    1
                ]
            },
            "area_7": {},
            "area_8": {
                "Wintersweet_rock_combination_B": [
                    "Plant",
                    1
                ],
                "Shrub_combination_A": [
                    "Plant",
                    2
                ]
            },
            "area_9": {
                "House_N_F": [
                    "House",
                    1
                ],
                "Pavilion_F": [
                    "Pavilion",
                    1
                ]
            },
            "area_10": {
                "House_N_E": [
                    "House",
                    1
                ],
                "Pavilion_D": [
                    "Pavilion",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ]
            },
            "area_11": {},
            "area_12": {
                "House_N_E": [
                    "House",
                    2
                ],
                "House_N_D": [
                    "House",
                    1
                ],
                "Rock_musa_combination_C": [
                    "Rock",
                    2
                ],
                "Pavilion_C": [
                    "Pavilion",
                    1
                ]
            },
            "area_13": {
                "House_N_D": [
                    "House",
                    1
                ],
                "Pavilion_E": [
                    "Pavilion",
                    1
                ]
            },
            "area_14": {},
            "area_15": {
                "Waterside_rock_combination_E": [
                    "Rock",
                    1
                ]
            },
            "area_16": {
                "House_N_E": [
                    "House",
                    2
                ],
                "Pavilion_D": [
                    "Pavilion",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ],
                "Rock_musa_combination_A": [
                    "Rock",
                    1
                ]
            },
            "area_17": {
                "House_N_C": [
                    "House",
                    1
                ],
                "Pavilion_A": [
                    "Pavilion",
                    1
                ],
                "Stone_combination_B": [
                    "Rock",
                    1
                ]
            },
            "area_18": {
                "Pavilion_C": [
                    "Pavilion",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ]
            },
            "area_19": {
                "Waterside_rock_combination_C": [
                    "Rock",
                    1
                ]
            },
            "area_20": {
                "Wintersweet_rock_combination_B": [
                    "Plant",
                    1
                ],
                "Stone_combination_B": [
                    "Rock",
                    2
                ]
            },
            "area_21": {
                "Pavilion_B": [
                    "Pavilion",
                    1
                ],
                "Bamboo_rock_combination_C": [
                    "Plant",
                    1
                ]
            },
            "area_22": {
                "Waterside_rock_combination_A": [
                    "Rock",
                    2
                ]
            },
            "area_23": {
                "Waterside_rock_combination_E": [
                    "Rock",
                    1
                ]
            },
            "area_24": {
                "Waterside_rock_combination_F": [
                    "Rock",
                    2
                ]
            },
            "area_25": {
                "Waterside_rock_combination_C": [
                    "Rock",
                    1
                ]
            },
            "area_26": {
                "House_N_D": [
                    "House",
                    1
                ],
                "Pavilion_F": [
                    "Pavilion",
                    1
                ]
            },
            "area_27": {
                "House_S_A": [
                    "House",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ]
            },
            "area_28": {
                "Wintersweet_rock_combination_A": [
                    "Plant",
                    2
                ]
            },
            "area_29": {
                "Pavilion_C": [
                    "Pavilion",
                    1
                ],
                "Bamboo_rock_combination_A": [
                    "Plant",
                    1
                ]
            },
            "area_30": {
                "Waterside_rock_combination_D": [
                    "Rock",
                    2
                ]
            },
            "area_31": {},
            "area_32": {
                "Bamboo_rock_combination_B": [
                    "Plant",
                    4
                ]
            },
            "area_33": {
                "House_N_D": [
                    "House",
                    1
                ]
            },
            "area_34": {
                "Waterside_rock_combination_A": [
                    "Rock",
                    1
                ]
            },
            "area_35": {},
            "area_36": {
                "Plant_combination_A": [
                    "Plant",
                    8
                ]
            },
            "area_37": {},
            "area_38": {
                "House_S_A": [
                    "House",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ],
                "Stone_combination_B": [
                    "Rock",
                    1
                ]
            },
            "area_39": {
                "Pavilion_A": [
                    "Pavilion",
                    1
                ],
                "Stone_combination_B": [
                    "Rock",
                    2
                ]
            },
            "area_40": {
                "House_N_F": [
                    "House",
                    1
                ],
                "Rock_Pine_combination_A": [
                    "Rock",
                    1
                ],
                "Bamboo_rock_combination_A": [
                    "Plant",
                    2
                ]
            },
            "area_41": {
                "House_N_D": [
                    "House",
                    1
                ],
                "Pavilion_E": [
                    "Pavilion",
                    1
                ]
            },
            "area_42": {
                "Rock_musa_combination_A": [
                    "Rock",
                    2
                ]
            },
            "area_43": {
                "Waterside_rock_combination_C": [
                    "Rock",
                    1
                ]
            },
            "area_44": {
                "Waterside_rock_combination_B": [
                    "Rock",
                    1
                ]
            },
            "area_45": {},
            "area_46": {
                "Waterside_rock_combination_F": [
                    "Rock",
                    1
                ]
            },
            "area_47": {
                "Bamboo_rock_combination_C": [
                    "Plant",
                    2
                ]
            },
            "area_48": {
                "House_N_C": [
                    "House",
                    1
                ],
                "Pavilion_E": [
                    "Pavilion",
                    1
                ]
            }
        }

    use_solver = False    
    use_random = False
    if args.use_conlan: use_original_conlan = True
    elif args.use_conlan_layout: use_original_conlan = True
    else: use_solver = True

    if use_solver:
        # query constraints
        #get_solver = False
        get_solver = False
        if get_solver:
            object_constraints = {}
            constraint_prompts = object_constraints_prompt.replace("*{area}*", str(area_info)).replace("*{object_dict}*", str(object_selection)).replace("*{text}*", text)
            constraint_response = constraint_setter(constraint_prompts)
            object_constraints = parse_constraint_setter_gptres(constraint_response)
        else:
            object_constraints =   {
                "area_0": {
                    "Taihu_rock_combination_A-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ]
                },
                "area_1": {
                    "House_N_E-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ],
                    "House_N_C-0": [
                        [
                            "aligned",
                            "House_N_E-0",
                            "alignment"
                        ]
                    ],
                },
                "area_2": {
                    "Pavilion_D-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ],
                    "Pavilion_D-1": [
                        [
                            "aligned",
                            "Pavilion_D-0",
                            "alignment"
                        ]
                    ],
                    "Waterside_rock_combination_E-0": [
                        [
                            "around",
                            "Pavilion_D-0",
                            "position"
                        ]
                    ]
                },
                "area_3": {
                    "Pavilion_E-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ],
                },
                "area_4": {},
                "area_5": {
                    "House_N_D-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ]
                },
                "area_6": {
                    "House_S_A-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ],
                    "House_S_A-1": [
                        [
                            "aligned",
                            "House_S_A-0",
                            "alignment"
                        ]
                    ],
                },
                "area_7": {
                },
                "area_8": {},
                "area_9": {
                    "Taihu_rock_combination_A-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ]
                },
                "area_10": {},
                "area_11": {},
                "area_12": {

                },
                "area_13": {
                    "Rock_bamboo_combination_A-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ]
                },
                "area_14": {},
                "area_15": {},
                "area_16": {

                },
                "area_17": {

                },
                "area_18": {},
                "area_19": {
                    "Pavilion_D-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ],

                },
                "area_20": {

                },
                "area_21": {},
                "area_22": {},
                "area_23": {
                    "Taihu_rock_combination_A-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ]
                },
                "area_24": {
                    "House_N_D-0": [
                        [
                            "middle",
                            "global"
                        ]
                    ]
                },
                "area_25": {

                },
                "area_26": {},
                "area_27": {},
                "area_28": {},
                "area_29": {},
                "area_30": {},
                "area_31": {},
                "area_32": {},
                "area_33": {},
            }

        for area, area_name in tqdm(zip(areas, object_constraints)):
            area_type, circuit = area
            area_constraints = object_constraints[area_name]
            #if len(area_constraints) == 0: continue
            # parse_constraint()
            # to (object_name, (obj_length, obj_width))
            objects_list = convert_constraints(area_constraints)
            if len(circuit) < 3:
                # for i in range(1000):
                #     print("-----------------")
                # print(circuit)
                # for edge_group in edges:
                #     edge_idxs, info = edge_group
                #     print(edge_idxs)
                continue
            poly = Polygon([points[idx] for idx in circuit])
            # NOTE remove the boundary buffer area
            buffer = None
            for i in range(len(circuit)):
                idx1, idx2 = circuit[i], circuit[(i + 1) % len(circuit)]
                if idx1 == idx2:
                    continue
                p1, p2 = p(points[idx1]), p(points[idx2])
                if (idx1, idx2) in edge2info:
                    edge_info = edge2info[(idx1, idx2)]
                    if 3 in edge_info:      continue
                    if 5 in edge_info:      buffer = bridge_buffer
                    elif 0 in edge_info:    buffer = wall_buffer
                    elif 4 in edge_info:    buffer = lakeside_buffer
                    elif 1 in edge_info:    buffer = MAIN_ROAD_WIDTH / 2    # 2
                    else:                   buffer = SUB_ROAD_WIDTH         # 2
                else:
                    continue
                tang = p2 - p1
                tang = tang / np.linalg.norm(tang)
                norm = p(p2[1] - p1[1], p1[0] - p2[0])
                norm = norm / np.linalg.norm(norm)
                if (tang is None) or (norm is None):
                    continue
                diff_poly = Polygon(
                    [
                        p1 + norm * buffer - tang * 0.5,
                        p1 - norm * buffer - tang * 0.5,
                        p2 - norm * buffer + tang * 0.5,
                        p2 + norm * buffer + tang * 0.5,
                    ]
                )
                poly = poly.difference(diff_poly) # remove the buffer area
            poly = poly.simplify(0, False) # strongly simplify
            polys = [poly] if isinstance(poly, Polygon) else list(poly.geoms) # single polygon or multiple polygons

            # NOTE GET 
            for poly in polys:
                if poly.area < 1:
                    continue
                if area_type == 1:      # lotus group
                    #if random.random() < 0.5:
                    lotus_data_gv = []
                    lotus_data_gv.append(find_data("Lotus_dense", plant_data_gv))
                    obj_infos = add_lotus_group(height_map, poly, lotus_data_gv)
                    all_obj_infos += obj_infos
                    if len(objects_list) == 0: continue
                elif area_type == 2:      # lotus
                    lotus_dense_data_gv = []
                    lotus_dense_data_gv.append(find_data("Lotus_dense", plant_data_gv))
                    obj_infos = add_lotus_group(height_map, poly, lotus_dense_data_gv)
                    all_obj_infos += obj_infos
                    if len(objects_list) == 0: continue
                elif area_type == 3:    # lake rock
                    lotus_data_gv = []
                    lotus_data_gv.append(find_data("Lotus_dense", plant_data_gv))
                    obj_infos = add_lotus_group(height_map, poly, lotus_data_gv)
                    obj_infos = add_lakerock(height_map, poly, lake_rock_data)
                    all_obj_infos += obj_infos
                    if len(objects_list) == 0: continue
                # elif area_type == 4:    # land
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)                        
                # elif area_type == 5:    # bushes or tree
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)
                # elif area_type == 6:    # bamboos
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)
                # elif area_type == 7:    # tree
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)
                # elif area_type == 8:    # pavilion
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)
                # elif area_type == 9:    # plaza
                #     labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                #     obj_infos = add_hugetree(height_map, poly, hugetree_data)
                #     all_obj_infos += obj_infos
                # elif area_type == 10:   # flower/bush beds
                #     labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                #     obj_infos = None
                #     if random.random() < 0.4 and poly.area < 800:
                #         obj_infos = add_plantbeds(terrain_labels, height_map, poly, flower_data, bush_data)
                #     elif random.random() < 0.6:
                #         obj_infos = add_building(height_map, poly, building_data, statue_data, bigtree_data)
                #     else:
                #         obj_infos = add_rockmaze(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                #     all_obj_infos += obj_infos
                # elif area_type == 11:   # tree lines
                #     labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                #     obj_infos = add_treelines(terrain_labels, height_map, poly, bigtree_data)
                #     all_obj_infos += obj_infos
                # #-------------------------
                # elif area_type == 12:   # building
                #     labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                #     obj_infos = add_building(height_map, poly, building_data, statue_data, bigtree_data)
                #     all_obj_infos += obj_infos
                # #-------------------------
                # elif area_type == 9 or area_type == 10 or area_type == 11 or area_type == 12:
                #     #labeling_area(terrain_labels, height_map, poly, 3, 0.2)
                # elif area_type == 13:   # hill rock
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)
                # elif area_type == 14:   # plants and bushes
                #     #labeling_area(terrain_labels, height_map, poly, 4, 1)

                solver = DFS_Solver(
                    grid_size=0.25, height_map = height_map, max_duration=10, constraint_bouns=1
                )
                obj_infos, remaining_area = solver.get_solution(poly, objects_list, area_constraints)

                # Judge if it get the full use of area
                """                allocated_union = unary_union(allocated_poly)
                remaining_polys = poly.difference(allocated_union)
                if isinstance(remaining_polys, MultiPolygon): # get the polygon with most space
                    #remaining_poly = unary_union(remaining_polys)
                    remaining_poly = None
                    max_area = -1
                    for mp in remaining_polys:
                        current_area = mp.area
                        if current_area > max_area:
                            max_area = current_area
                            remaining_poly = mp"""
                if len(obj_infos) == 0: # no object is placed 
                    # fill some thing, if the area is empty
                    # try to place architecture
                    if area_type == 5:    # bushes
                        #labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_trees_gv(height_map, poly, plant_data_gv)
                        all_obj_infos += obj_infos
                        # obj_infos = add_bushes(height_map, poly, rectbush_data, bigbush_data)
                        # obj_infos2 = add_few_trees(height_map, poly, bigtree_data)
                        # all_obj_infos += obj_infos
                        # all_obj_infos += obj_infos2
                    elif area_type == 6:    # bamboos
                        #labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_bamboos(height_map, poly, bamboo_data)
                        all_obj_infos += obj_infos
                    elif area_type == 7:    # tree
                        #labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_trees(height_map, poly, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 8:    # pavilion
                        #labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_rest_pavilion(terrain_labels, height_map, poly, pavilion_data_gv, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 9 or area_type == 10 or area_type == 11 or area_type == 12:
                        #labeling_area(terrain_labels, height_map, poly, 3, 0.2)
                        building_data_gv = None
                        building_type = None
                        if poly.area > 400:     
                            if random.random() > 0.4: 
                                building_data_gv = attic_data_gv
                                building_type = "Attic"
                            else:
                                building_data_gv = house_data_gv
                                if random.random() > 0.5: building_type = "House_S"
                                else: building_type = "House_N"
                        elif poly.area > 300:   
                            building_data_gv = house_data_gv
                            building_type = "House_N"
                        elif poly.area > 100: 
                            building_data_gv = house_data_gv
                            building_type = "House_N"
                        else: 
                            building_data_gv = pavilion_data_gv
                            building_type = "Pavilion"
                        obj_infos, allocated_poly, allocated_area = add_building_by_optimization(height_map, 
                                poly, building_data_gv, plant_data_gv, rock_data_gv, bigtree_data, building_type, water_poly, terrain_labels, pavilion_data_gv)
                        # if (poly.area - allocated_area)/poly.area > 0.3:
                        #     building_data_gv = pavilion_data_gv
                        #     building_type = "Pavilion"
                        #     obj_infos, allocated_poly, allocated_area  = add_building_by_optimization(height_map, poly, building_data_gv, plant_data_gv, rock_data_gv, bigtree_data, building_type)
                        all_obj_infos += obj_infos
                        # if poly.distance(water_poly) < 1: #poly.touches(water_poly) : # if the area is connected to water area, set the view pavillion
                        #     #pavillion_view_data = find_data("Pavillion_B", pavilion_data)
                        #     allocated_union = unary_union(allocated_poly)
                        #     remaining_polys = poly.difference(allocated_union)
                        #     if isinstance(remaining_polys, MultiPolygon): # get the polygon with most space
                        #         #remaining_poly = unary_union(remaining_polys)
                        #         remaining_poly = None
                        #         max_area = -1
                        #         for mp in remaining_polys:
                        #             current_area = mp.area
                        #             if current_area > max_area:
                        #                 max_area = current_area
                        #                 remaining_poly = mp

                        #     else: remaining_poly = remaining_polys
                        #     obj_infos = add_view_pavilion(terrain_labels, height_map, remaining_poly, water_poly, pavilion_data_gv, bigtree_data, constraints)
                        #     all_obj_infos += obj_infos

                elif remaining_area.area / poly.area > 0.3: # few objects is placed
                    # fill some thing, if the area is empty
                    # try to place architecture
                    if area_type == 5:    # bushes
                        #labeling_area(terrain_labels, height_map, remaining_area, 4, 1)
                        obj_infos = add_trees_gv(height_map, remaining_area, plant_data_gv)
                        all_obj_infos += obj_infos
                        # obj_infos = add_bushes(height_map, poly, rectbush_data, bigbush_data)
                        # obj_infos2 = add_few_trees(height_map, poly, bigtree_data)
                        # all_obj_infos += obj_infos
                        # all_obj_infos += obj_infos2
                    elif area_type == 6:    # bamboos
                        #labeling_area(terrain_labels, height_map, remaining_area, 4, 1)
                        obj_infos = add_bamboos(height_map, remaining_area, bamboo_data)
                        all_obj_infos += obj_infos
                    elif area_type == 7:    # tree
                        #labeling_area(terrain_labels, height_map, remaining_area, 4, 1)
                        obj_infos = add_trees(height_map, remaining_area, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 8:    # pavilion
                        #labeling_area(terrain_labels, height_map, remaining_area, 4, 1)
                        obj_infos = add_rest_pavilion(terrain_labels, height_map, remaining_area, pavilion_data_gv, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 9 or area_type == 10 or area_type == 11 or area_type == 12:
                        #labeling_area(terrain_labels, height_map, poly, 3, 0.2)
                        building_data_gv = None
                        building_type = None
                        if poly.area > 400:     
                            if random.random() > 0.4: 
                                building_data_gv = attic_data_gv
                                building_type = "Attic"
                            else:
                                building_data_gv = house_data_gv
                                if random.random() > 0.5: building_type = "House_S"
                                else: building_type = "House_N"
                        elif poly.area > 300:   
                            building_data_gv = house_data_gv
                            building_type = "House_N"
                        elif poly.area > 100: 
                            building_data_gv = house_data_gv
                            building_type = "House_N"
                        else: 
                            building_data_gv = pavilion_data_gv
                            building_type = "Pavilion"
                        obj_infos, allocated_poly, allocated_area = add_building_by_optimization(height_map, 
                                poly, building_data_gv, plant_data_gv, rock_data_gv, bigtree_data, building_type, water_poly, terrain_labels, pavilion_data_gv)
                        all_obj_infos += obj_infos
                else:
                    all_obj_infos += obj_infos
    elif use_random:
        for area, area_name in tqdm(zip(areas, object_constraints)):
            area_type, circuit = area
            area_constraints = object_constraints[area_name]
    elif use_original_conlan:
        for area in tqdm(areas):
            area_type, circuit = area
            if len(circuit) < 3:
                # for i in range(1000):
                #     print("-----------------")
                # print(circuit)
                # for edge_group in edges:
                #     edge_idxs, info = edge_group
                #     print(edge_idxs)
                continue
            poly = Polygon([points[idx] for idx in circuit])
            buffer = None
            for i in range(len(circuit)):
                idx1, idx2 = circuit[i], circuit[(i + 1) % len(circuit)]
                if idx1 == idx2:
                    continue
                p1, p2 = p(points[idx1]), p(points[idx2])
                if (idx1, idx2) in edge2info:
                    edge_info = edge2info[(idx1, idx2)]
                    if 3 in edge_info:
                        continue
                    if 5 in edge_info:
                        buffer = bridge_buffer
                    elif 0 in edge_info:
                        buffer = wall_buffer
                    elif 4 in edge_info:
                        buffer = lakeside_buffer
                    elif 1 in edge_info:
                        buffer = MAIN_ROAD_WIDTH / 2
                    else:
                        buffer = SUB_ROAD_WIDTH
                else:
                    continue
                tang = p2 - p1
                tang = tang / np.linalg.norm(tang)
                norm = p(p2[1] - p1[1], p1[0] - p2[0])
                norm = norm / np.linalg.norm(norm)
                if (tang is None) or (norm is None):
                    continue
                diff_poly = Polygon(
                    [
                        p1 + norm * buffer - tang * 0.5,
                        p1 - norm * buffer - tang * 0.5,
                        p2 - norm * buffer + tang * 0.5,
                        p2 + norm * buffer + tang * 0.5,
                    ]
                )
                poly = poly.difference(diff_poly)
            poly = poly.simplify(0, False)
            polys = [poly] if isinstance(poly, Polygon) else list(poly.geoms)
            for poly in polys:
                if poly.area < 1:
                    continue

                if area_type == 2:  # lotus
                    obj_infos = add_lotus(height_map, poly, lotus_data)
                    all_obj_infos += obj_infos
                elif area_type == 3:  # lake rock
                    obj_infos = add_lakerock(height_map, poly, lake_rock_data)
                    all_obj_infos += obj_infos
                elif area_type == 4:  # land
                    labeling_area(terrain_labels, height_map, poly, 4, 1)
                    obj_infos = add_rockmaze(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                    all_obj_infos += obj_infos
                elif area_type == 5:  # bushes
                    obj_infos = add_bushes(height_map, poly, rectbush_data, bigbush_data)
                    obj_infos2 = add_few_trees(height_map, poly, bigtree_data)
                    all_obj_infos += obj_infos
                    all_obj_infos += obj_infos2
                elif area_type == 6:  # bamboos
                    labeling_area(terrain_labels, height_map, poly, 4, 1)
                    obj_infos = add_bamboos(height_map, poly, bamboo_data)
                    all_obj_infos += obj_infos
                elif area_type == 7:  # tree
                    labeling_area(terrain_labels, height_map, poly, 4, 1)
                    obj_infos = add_trees(height_map, poly, bigtree_data)
                    all_obj_infos += obj_infos
                elif area_type == 8:  # pavilion
                    labeling_area(terrain_labels, height_map, poly, 4, 1)
                    obj_infos = add_pavilion_gv(terrain_labels, height_map, poly, pavilion_data_gv, bigtree_data, plant_data_gv)
                    all_obj_infos += obj_infos
                elif area_type == 9:  # plaza
                    labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                    obj_infos = add_hugetree(height_map, poly, hugetree_data)
                    all_obj_infos += obj_infos
                elif area_type == 10:  # flower/bush beds
                    labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                    obj_infos = None
                    main_building_data = []
                    building_data_gv = pavilion_data_gv
                    building_type_list = ["Attic","House_S","House_N","Pavilion"]
                    building_type = random.choice(building_type_list)
                    if building_type == "Attic":
                        building_data_gv = attic_data_gv
                    elif building_type == "House_S":
                        building_data_gv = house_data_gv
                    elif building_type == "House_N":
                        building_data_gv = house_data_gv

                    if random.random() < 0.4 and poly.area < 800:
                        obj_infos = add_plantbeds(terrain_labels, height_map, poly, flower_data, bush_data)
                    elif random.random() < 0.6:
                        obj_infos = add_building(height_map, poly, building_data_gv, statue_data, bigtree_data, plant_data_gv, building_type)
                    else:
                        obj_infos = add_rockmaze(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                    all_obj_infos += obj_infos
                elif area_type == 11:  # tree lines
                    labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                    obj_infos = add_treelines(terrain_labels, height_map, poly, bigtree_data)
                    all_obj_infos += obj_infos
                elif area_type == 12:  # building
                    labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                    main_building_data = []
                    building_data_gv = pavilion_data_gv
                    building_type_list = ["Attic","House_S","House_N","Pavilion"]
                    building_type = random.choice(building_type_list)
                    if building_type == "Attic":
                        building_data_gv = attic_data_gv
                    elif building_type == "House_S":
                        building_data_gv = house_data_gv
                    elif building_type == "House_N":
                        building_data_gv = house_data_gv
                    obj_infos = add_building(height_map, poly, building_data_gv, statue_data, bigtree_data, plant_data_gv, building_type)
                    all_obj_infos += obj_infos
                elif area_type == 13:  # hill rock
                    labeling_area(terrain_labels, height_map, poly, 4, 1)
                    obj_infos = add_hillrock(height_map, poly, hill_rock_data)
                    obj_infos2 = add_hugetree(height_map, poly, hugetree_data)
                    all_obj_infos += obj_infos
                    all_obj_infos += obj_infos2
                elif area_type == 14:  # plants and bushes
                    labeling_area(terrain_labels, height_map, poly, 4, 1)
                    obj_infos = add_plants(height_map, poly, plant_data, plantbush_data)
                    obj_infos2 = add_few_trees(height_map, poly, bigtree_data)
                    all_obj_infos += obj_infos
                    all_obj_infos += obj_infos2
    else:
        for area in tqdm(areas):
            area_type, circuit = area
            if len(circuit) < 3:
                # for i in range(1000):
                #     print("-----------------")
                # print(circuit)
                # for edge_group in edges:
                #     edge_idxs, info = edge_group
                #     print(edge_idxs)
                continue
            poly = Polygon([points[idx] for idx in circuit])
            # NOTE remove the boundary buffer area
            buffer = None
            for i in range(len(circuit)):
                idx1, idx2 = circuit[i], circuit[(i + 1) % len(circuit)]
                if idx1 == idx2:
                    continue
                p1, p2 = p(points[idx1]), p(points[idx2])
                if (idx1, idx2) in edge2info:
                    edge_info = edge2info[(idx1, idx2)]
                    if 3 in edge_info:      continue
                    if 5 in edge_info:      buffer = bridge_buffer
                    elif 0 in edge_info:    buffer = wall_buffer
                    elif 4 in edge_info:    buffer = lakeside_buffer
                    elif 1 in edge_info:    buffer = MAIN_ROAD_WIDTH / 2    # 2
                    else:                   buffer = SUB_ROAD_WIDTH         # 2
                else:
                    continue
                tang = p2 - p1
                tang = tang / np.linalg.norm(tang)
                norm = p(p2[1] - p1[1], p1[0] - p2[0])
                norm = norm / np.linalg.norm(norm)
                if (tang is None) or (norm is None):
                    continue
                diff_poly = Polygon(
                    [
                        p1 + norm * buffer - tang * 0.5,
                        p1 - norm * buffer - tang * 0.5,
                        p2 - norm * buffer + tang * 0.5,
                        p2 + norm * buffer + tang * 0.5,
                    ]
                )
                poly = poly.difference(diff_poly) # remove the buffer area
            poly = poly.simplify(0, False) # strongly simplify
            polys = [poly] if isinstance(poly, Polygon) else list(poly.geoms) # single polygon or multiple polygons

            # NOTE GET 

            for poly in polys:
                # single polygon in this loop
                if poly.area < 1:
                    continue
                use_solver = True
                use_random = False
    
                if use_random:
                    obj_infos = random_placing()
                    all_obj_infos += obj_infos
                else:
                    # Hard PCG
                    if area_type == 1:      # lotus group
                       if random.random() < 0.5:
                           obj_infos = add_lotus_group(height_map, poly, lotus_data)
                           all_obj_infos += obj_infos
                    elif area_type == 2:      # lotus
                        obj_infos = add_lotus(height_map, poly, lotus_data)
                        all_obj_infos += obj_infos
                    elif area_type == 3:    # lake rock
                        obj_infos = add_lakerock(height_map, poly, lake_rock_data)
                        all_obj_infos += obj_infos
                    elif area_type == 4:    # land
                        labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_rockmaze(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 5:    # bushes
                        obj_infos = add_bushes(height_map, poly, rectbush_data, bigbush_data)
                        obj_infos2 = add_few_trees(height_map, poly, bigtree_data)
                        all_obj_infos += obj_infos
                        all_obj_infos += obj_infos2
                    elif area_type == 6:    # bamboos
                        labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_bamboos(height_map, poly, bamboo_data)
                        all_obj_infos += obj_infos
                    elif area_type == 7:    # tree
                        labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_trees(height_map, poly, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 8:    # pavilion
                        labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_rest_pavilion(terrain_labels, height_map, poly, pavilion_data, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 9:    # plaza
                        labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                        obj_infos = add_hugetree(height_map, poly, hugetree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 10:   # flower/bush beds
                        labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                        obj_infos = None
                        if random.random() < 0.4 and poly.area < 800:
                            obj_infos = add_plantbeds(terrain_labels, height_map, poly, flower_data, bush_data)
                        elif random.random() < 0.6:
                            obj_infos = add_building(height_map, poly, building_data, statue_data, bigtree_data)
                        else:
                            obj_infos = add_rockmaze(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                        all_obj_infos += obj_infos
                    elif area_type == 11:   # tree lines
                        labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                        obj_infos = add_treelines(terrain_labels, height_map, poly, bigtree_data)
                        all_obj_infos += obj_infos
                    #-------------------------
                    elif area_type == 12:   # building
                        labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                        obj_infos = add_building(height_map, poly, building_data, statue_data, bigtree_data)
                        all_obj_infos += obj_infos
                    #-------------------------
                    # elif area_type == 9 or area_type == 10 or area_type == 11 or area_type == 12:
                    #     labeling_area(terrain_labels, height_map, poly, 3, 0.2)
                    #     obj_infos, allocated_poly, allocated_area = add_building_by_optimization(height_map, poly, building_data, plant_data_gv, rock_data_gv, bigtree_data)
                    #     all_obj_infos += obj_infos
                    #     # if poly.distance(water_poly) < 5: #poly.touches(water_poly) : # if the area is connected to water area, set the view pavillion
                    #     #     #pavillion_view_data = find_data("Pavillion_B", pavilion_data)
                    #     #     allocated_union = unary_union(allocated_poly)
                    #     #     remaining_poly = poly.difference(allocated_union)
                    #     #     obj_infos = add_view_pavilion(terrain_labels, height_map, remaining_poly, water_poly, pavilion_data)
                    #     #     all_obj_infos += obj_infos
                    elif area_type == 13:   # hill rock
                        labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_hillrock(height_map, poly, hill_rock_data)
                        obj_infos2 = add_hugetree(height_map, poly, hugetree_data)
                        all_obj_infos += obj_infos
                        all_obj_infos += obj_infos2
                    elif area_type == 14:   # plants and bushes
                        labeling_area(terrain_labels, height_map, poly, 4, 1)
                        obj_infos = add_plants(height_map, poly, plant_data, plantbush_data)
                        obj_infos2 = add_few_trees(height_map, poly, bigtree_data)
                        all_obj_infos += obj_infos
                        all_obj_infos += obj_infos2
    
    view_points = decide_view_points(height_map, keyunits)
    output_height_map(height_map, MAP_W, MAP_H, "height_map_" + str(gen_idx), suffix)
    output_label_map(terrain_labels, MAP_W, MAP_H, "label_map_" + str(gen_idx), suffix)
    output_scene(all_obj_infos, view_points, data, garden_verse, gen_idx, suffix)
    output_visualize(all_obj_infos, data, garden_verse, points, edges, areas, infrastructure, "structure") # Visualize build reference
    if not args.use_conlan and not args.use_conlan_layout: output_json(object_selection, object_constraints, "llm_response")
    pathway_score(all_obj_infos, data, garden_verse, points, edges, areas, infrastructure, "pathway_score")

def add_rockmaze_random(
    height_map: List[List[float]],
    poly: Polygon,
    hill_rock_data: List[dict],
    lake_rock_data: List[dict],
    tree_data: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    hillrock_size_list = [(dat["size"][0], dat["size"][2]) for dat in hill_rock_data]
    lakerock_size_list = [(dat["size"][0], dat["size"][2]) for dat in lake_rock_data]
    tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_data]
    lakerock_point_and_types = random_placing(poly, lakerock_size_list, 0.05, True)
    hillrock_point_and_types = random_placing(poly, hillrock_size_list, 0.1, True)
    tree_point_and_types = random_placing(poly, tree_size_list, 0.2, True)
    hillrock_obj_infos_dict, lakerock_obj_infos_dict, tree_obj_infos_dict = {}, {}, {}
    for typ_idx in range(len(hill_rock_data)):
        hillrock_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(lake_rock_data)):
        lakerock_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(tree_data)):
        tree_obj_infos_dict[typ_idx] = []

    for point_and_type in hillrock_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT - random.uniform(0.2, 0.4)
        hillrock_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in lakerock_point_and_types:
        x, z, typ = point_and_type
        x += random.uniform(-1, 1)
        z += random.uniform(-1, 1)
        y = get_height(height_map, x, z) * MAX_HEIGHT - random.uniform(0.1, 0.3)
        lakerock_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in tree_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        tree_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    obj_infos_list = []
    for typ_idx in hillrock_obj_infos_dict:
        obj_infos_list.append((hillrock_obj_infos_dict[typ_idx], hill_rock_data[typ_idx]["name"]))
    for typ_idx in lakerock_obj_infos_dict:
        obj_infos_list.append((lakerock_obj_infos_dict[typ_idx], lake_rock_data[typ_idx]["name"]))
    for typ_idx in tree_obj_infos_dict:
        obj_infos_list.append((tree_obj_infos_dict[typ_idx], tree_data[typ_idx]["name"]))
    return obj_infos_list


def add_bushes_random(
    height_map: List[List[float]], poly: Polygon, rectbush_datas: List[dict], bigbush_datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    rectbush_size_list = [(dat["size"][0], dat["size"][2]) for dat in rectbush_datas]
    bigbush_size_list = [(dat["size"][0], dat["size"][2]) for dat in bigbush_datas]
    poly_area = poly.area
    point_and_types = None
    typeflag = 0
    point_and_types = random_placing(poly, rectbush_size_list, 0.3, True)
    obj_infos_dict = {}
    length = len(rectbush_datas) if typeflag == 0 else len(bigbush_datas)
    for typ_idx in range(length):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        if typeflag == 0:
            obj_infos_list.append((obj_infos_dict[typ_idx], rectbush_datas[typ_idx]["name"]))
        else:
            obj_infos_list.append((obj_infos_dict[typ_idx], bigbush_datas[typ_idx]["name"]))
    return obj_infos_list


def add_bamboos_random(
    height_map: List[List[float]], poly: Polygon, datas: List[dict]
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    size_list = [(dat["size"][0], dat["size"][2]) for dat in datas]
    poly_area = poly.area
    point_and_types = random_placing(poly, size_list, 0.6, True)
    obj_infos_dict = {}
    for typ_idx in range(len(datas)):
        obj_infos_dict[typ_idx] = []

    for point_and_type in point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))

    obj_infos_list = []
    for typ_idx in obj_infos_dict:
        obj_infos_list.append((obj_infos_dict[typ_idx], datas[typ_idx]["name"]))
    return obj_infos_list


def add_pavilion_random(
    terrain_labels: List[List[List[float]]],
    height_map: List[List[float]],
    poly: Polygon,
    pav_datas: List[dict],
    tree_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    pav_size_list = [(dat["size"][0], dat["size"][2]) for dat in pav_datas]
    tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    pav_point_and_types = random_placing(poly, pav_size_list, 0.25, True)
    tree_point_and_types = random_placing(poly, tree_size_list, 0.5, True)
    pav_obj_infos_dict, tree_obj_infos_dict = {}, {}
    for typ_idx in range(len(pav_datas)):
        pav_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(tree_datas)):
        tree_obj_infos_dict[typ_idx] = []

    for point_and_type in pav_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        pav_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in tree_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        tree_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    obj_infos_list = []
    for typ_idx in pav_obj_infos_dict:
        obj_infos_list.append((pav_obj_infos_dict[typ_idx], pav_datas[typ_idx]["name"]))
    for typ_idx in tree_obj_infos_dict:
        obj_infos_list.append((tree_obj_infos_dict[typ_idx], tree_datas[typ_idx]["name"]))

    return obj_infos_list


def add_plantbeds_random(
    terrain_labels: List[List[List[float]]],
    height_map: List[List[float]],
    poly: Polygon,
    flower_datas: List[dict],
    bush_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:
    flower_size_list, bush_size_list = [(dat["size"][0], dat["size"][2]) for dat in flower_datas], [
        (dat["size"][0], dat["size"][2]) for dat in bush_datas
    ]
    flower_point_and_types = random_placing(poly, flower_size_list, 0.3, True)
    bush_point_and_types = random_placing(poly, bush_size_list, 0.3, True)
    flower_obj_infos_dict, bush_obj_infos_dict = {}, {}
    for typ_idx in range(len(flower_datas)):
        flower_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(bush_datas)):
        bush_obj_infos_dict[typ_idx] = []

    for point_and_type in flower_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        flower_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in bush_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        bush_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    obj_infos_list = []
    for typ_idx in flower_obj_infos_dict:
        obj_infos_list.append((flower_obj_infos_dict[typ_idx], flower_datas[typ_idx]["name"]))
    for typ_idx in bush_obj_infos_dict:
        obj_infos_list.append((bush_obj_infos_dict[typ_idx], bush_datas[typ_idx]["name"]))
    return obj_infos_list


def add_building_random(
    height_map: List[List[float]],
    poly: Polygon,
    build_datas: List[dict],
    statue_datas: List[dict],
    tree_datas: List[dict],
) -> List[Tuple[List[Tuple[float, float, float, float]], str]]:  # TODO
    build_size_list = [(dat["size"][0], dat["size"][2]) for dat in build_datas]
    statue_size_list = [(dat["size"][0], dat["size"][2]) for dat in statue_datas]
    tree_size_list = [(dat["size"][0], dat["size"][2]) for dat in tree_datas]
    build_point_and_types = random_placing(poly, build_size_list, 0.3, True)
    statue_point_and_types = random_placing(poly, statue_size_list, 0.02, True)
    tree_point_and_types = random_placing(poly, tree_size_list, 0.05, True)
    build_obj_infos_dict, statue_obj_infos_dict, tree_obj_infos_dict = {}, {}, {}
    for typ_idx in range(len(build_datas)):
        build_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(statue_datas)):
        statue_obj_infos_dict[typ_idx] = []
    for typ_idx in range(len(tree_datas)):
        tree_obj_infos_dict[typ_idx] = []

    for point_and_type in build_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        build_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in statue_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        statue_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    for point_and_type in tree_point_and_types:
        x, z, typ = point_and_type
        y = get_height(height_map, x, z) * MAX_HEIGHT
        tree_obj_infos_dict[typ].append((x, y, z, random.uniform(0, 2 * pi)))
    obj_infos_list = []
    for typ_idx in build_obj_infos_dict:
        obj_infos_list.append((build_obj_infos_dict[typ_idx], build_datas[typ_idx]["name"]))
    for typ_idx in statue_obj_infos_dict:
        obj_infos_list.append((statue_obj_infos_dict[typ_idx], statue_datas[typ_idx]["name"]))
    for typ_idx in tree_obj_infos_dict:
        obj_infos_list.append((tree_obj_infos_dict[typ_idx], tree_datas[typ_idx]["name"]))
    return obj_infos_list


def pcg_random(
    height_map: List[List[float]],
    points: List[Tuple[float, float]],
    edges: List[Tuple[List[Tuple[int, int]], List[int]]],
    areas: List[Tuple[int, List[int]]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[Tuple[int, int], Tuple[int, int]]], dict],
    parameters: dict,
    data: dict,
    gen_idx: int,
    suffix: str = "",
) -> None:
    """
    points[i]: (x, y)
    edges[i]: (points, info) info: 0: wall, 1: main road, 2: secondary road, 3: type border, 4: lakeside, 5: bridge, 6: wall with hole
    edges[i][0][j]: (point_idx1, point_idx2)
    areas[i]: (type, circuit) type: 1: water, 2: lotus, 3: lake rock, 4: pure land, 5: grass/flower, 6: bush, 7: tree,
                                    8: pavilion, 9: plaza, 10: flowerbed, 11: tree lines, 12: building, 13: hill rock, 14: grass
    areas[i][1][j]: point_idx
    """
    logpath = "logs/" + str(gen_idx) + suffix + ".txt"
    sys.stdout = open(logpath, "a")
    sys.stderr = open(logpath, "a")

    entrance_points, keyunits, edge_used = infrastructure
    entrance_points = [(x * RL, y * RL) for x, y in entrance_points]
    points = [(x * RL, y * RL) for x, y in points]
    (
        lotus_data,
        plant_data,
        lake_rock_data,
        flower_data,
        plantbush_data,
        bush_data,
        bigbush_data,
        rectbush_data,
        bamboo_data,
        hugetree_data,
        bigtree_data,
        pavilion_data,
        building_data,
        hill_rock_data,
        wall_data,
        bridge_data,
        statue_data,
    ) = (
        data["lotus"],
        data["plant"],
        data["lake_rock"],
        data["flower"],
        data["plantbush"],
        data["bush"],
        data["bigbush"],
        data["rectbush"],
        data["bamboo"],
        data["hugetree"],
        data["bigtree"],
        data["pavilion"],
        data["building"],
        data["hill_rock"],
        data["wall"],
        data["bridge"],
        data["statue"],
    )
    terrain_labels = np.zeros((MAP_W, MAP_H, 5))  # 0: none, 1: main road, 2: secondary road, 3: plaza, 4: land
    all_obj_infos = []

    edge2info = {}
    for edge_group in edges:
        edge_idxs, info = edge_group
        for i in range(len(edge_idxs)):
            edge2info[edge_idxs[i]] = info
            edge2info[(edge_idxs[i][1], edge_idxs[i][0])] = info

    print("handle edges:")
    for edge_group in tqdm(edges):
        edge_idxs, info = edge_group
        for i in range(len(edge_idxs) - 1):
            if edge_idxs[i][1] != edge_idxs[i + 1][0]:
                raise ValueError("Edge not connected")
        point_idxs = [idx[0] for idx in edge_idxs] + [edge_idxs[-1][1]]
        point_locs = [points[idx] for idx in point_idxs]
        rand_num = random.random()
        total_length = 0
        for i in range(len(point_locs) - 1):
            total_length += eu_dist(point_locs[i], point_locs[i + 1])
        if 3 in info:  # type border
            pass
        elif 5 in info:  # bridge
            longbridge_data, zigzagbridge_data = find_data("Bridge", bridge_data), find_data(
                "Zigzag_Bridge", bridge_data
            )
            if (2 in info) and rand_num < 0.7:
                obj_infos = build_zigzagbridge(height_map, point_locs, zigzagbridge_data)
                all_obj_infos.append((obj_infos, "Zigzag_Bridge"))
            else:
                obj_infos, road_idxs = build_bridge(height_map, point_locs, longbridge_data)
                all_obj_infos.append((obj_infos, "Bridge"))
                for idx in road_idxs:
                    terrain_labels[idx[0]][idx[1]][1] += 1
        elif 6 in info:  # entrance
            wall4_data = find_data("Wall_400x300", wall_data)
            obj_infos = build_entrance(height_map, point_locs, wall4_data, entrance_points)
            all_obj_infos.append((obj_infos, "Wall_400x300"))
        elif 0 in info:  # wall
            wall4_data = find_data("Wall_400x300", wall_data)
            obj_infos = build_wall(height_map, point_locs, wall4_data)
            all_obj_infos.append((obj_infos, "Wall_400x300"))
        elif (1 in info) or (2 in info):  # road
            poly = None
            for i in range(len(point_locs) - 1):
                sp, ep = p(point_locs[i]), p(point_locs[i + 1])
                tang = p(ep[0] - sp[0], ep[1] - sp[1])
                tang = tang / np.linalg.norm(tang)
                norm = p(ep[1] - sp[1], sp[0] - ep[0])
                norm = norm / np.linalg.norm(norm)
                road_width = MAIN_ROAD_WIDTH if (1 in info) else SUB_ROAD_WIDTH
                new_poly = Polygon(
                    [
                        sp + norm * road_width / 2 - tang * 1e-3,
                        sp - norm * road_width / 2 - tang * 1e-3,
                        ep - norm * road_width / 2 + tang * 1e-3,
                        ep + norm * road_width / 2 + tang * 1e-3,
                    ]
                )
                if poly is None:
                    poly = new_poly
                else:
                    poly = poly.union(new_poly)
            poly = poly.simplify(0, False)
            labeling_area(terrain_labels, height_map, poly, 1 if (1 in info) else 2, 1)

    lakeside_buffer = 3
    bridge_buffer = 3
    wall_buffer = 0.5

    print("handle areas:")
    for area in tqdm(areas):
        area_type, circuit = area
        if len(circuit) < 3:
            continue
        poly = Polygon([points[idx] for idx in circuit])
        buffer = None
        for i in range(len(circuit)):
            idx1, idx2 = circuit[i], circuit[(i + 1) % len(circuit)]
            if idx1 == idx2:
                continue
            p1, p2 = p(points[idx1]), p(points[idx2])
            if (idx1, idx2) in edge2info:
                edge_info = edge2info[(idx1, idx2)]
                if 3 in edge_info:
                    continue
                if 5 in edge_info:
                    buffer = bridge_buffer
                elif 0 in edge_info:
                    buffer = wall_buffer
                elif 4 in edge_info:
                    buffer = lakeside_buffer
                elif 1 in edge_info:
                    buffer = MAIN_ROAD_WIDTH / 2
                else:
                    buffer = SUB_ROAD_WIDTH
            else:
                continue
            tang = p2 - p1
            tang = tang / np.linalg.norm(tang)
            norm = p(p2[1] - p1[1], p1[0] - p2[0])
            norm = norm / np.linalg.norm(norm)
            if (tang is None) or (norm is None):
                continue
            diff_poly = Polygon(
                [
                    p1 + norm * buffer - tang * 0.5,
                    p1 - norm * buffer - tang * 0.5,
                    p2 - norm * buffer + tang * 0.5,
                    p2 + norm * buffer + tang * 0.5,
                ]
            )
            poly = poly.difference(diff_poly)
        poly = poly.simplify(0, False)
        polys = [poly] if isinstance(poly, Polygon) else list(poly.geoms)
        for poly in polys:
            if poly.area < 1:
                continue

            if area_type == 2:  # lotus
                obj_infos = add_lotus(height_map, poly, lotus_data)
                all_obj_infos += obj_infos
            elif area_type == 3:  # lake rock
                obj_infos = add_lakerock(height_map, poly, lake_rock_data)
                all_obj_infos += obj_infos
            elif area_type == 4:  # land
                labeling_area(terrain_labels, height_map, poly, 4, 1)
                obj_infos = add_rockmaze_random(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                all_obj_infos += obj_infos
            elif area_type == 5:  # bushes
                obj_infos = add_bushes_random(height_map, poly, rectbush_data, bigbush_data)
                all_obj_infos += obj_infos
            elif area_type == 6:  # bamboos
                labeling_area(terrain_labels, height_map, poly, 4, 1)
                obj_infos = add_bamboos_random(height_map, poly, bamboo_data)
                all_obj_infos += obj_infos
            elif area_type == 7:  # tree
                labeling_area(terrain_labels, height_map, poly, 4, 1)
                obj_infos = add_trees(height_map, poly, bigtree_data)
                all_obj_infos += obj_infos
            elif area_type == 8:  # pavilion
                labeling_area(terrain_labels, height_map, poly, 4, 1)
                obj_infos = add_pavilion_random(terrain_labels, height_map, poly, pavilion_data, bigtree_data)
                all_obj_infos += obj_infos
            elif area_type == 9:  # plaza
                labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                obj_infos = add_hugetree(height_map, poly, hugetree_data)
                all_obj_infos += obj_infos
            elif area_type == 10:  # flower/bush beds
                labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                obj_infos = None
                if random.random() < 0.5:
                    obj_infos = add_plantbeds_random(terrain_labels, height_map, poly, flower_data, bush_data)
                else:
                    obj_infos = add_rockmaze_random(height_map, poly, hill_rock_data, lake_rock_data, bigtree_data)
                all_obj_infos += obj_infos
            elif area_type == 11:  # tree lines
                labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                obj_infos = add_trees(height_map, poly, bigtree_data)
                all_obj_infos += obj_infos
            elif area_type == 12:  # building
                labeling_area(terrain_labels, height_map, poly, 3, 0.5)
                obj_infos = add_building_random(height_map, poly, building_data, statue_data, bigtree_data)
                all_obj_infos += obj_infos
            elif area_type == 13:  # hill rock
                labeling_area(terrain_labels, height_map, poly, 4, 1)
                obj_infos = add_hillrock(height_map, poly, hill_rock_data)
                all_obj_infos += obj_infos
            elif area_type == 14:  # plants and bushes
                labeling_area(terrain_labels, height_map, poly, 4, 1)
                obj_infos = add_plants(height_map, poly, plant_data, plantbush_data)
                all_obj_infos += obj_infos

    view_points = decide_view_points(height_map, keyunits)
    output_height_map(height_map, MAP_W, MAP_H, "height_map_" + str(gen_idx), suffix)
    output_label_map(terrain_labels, MAP_W, MAP_H, "label_map_" + str(gen_idx), suffix)
    output_scene(all_obj_infos, view_points, data, gen_idx, suffix)




# Asset Selector，constraint_setter

def asset_selector(input_prompts: str):
    response = client.responses.create(
        model="gpt-4.1",
        input=input_prompts,
        tools=[{
            "type": "file_search",
            "vector_store_ids": ["vs_6820650b99c88191beb65d5a04c01bbd"]
        }]
    )
    text_response = None
    for output in response.output:
        if output.type == "message":
            for content_part in output.content:
                if content_part.type == "output_text":
                    text_response = content_part.text
                    break  # found it, no need to continue
        if text_response:
            break

    if text_response:
        print("\nasset selector response:\n")
        print(text_response)
    else:
        print("asset selector agent error!")
    return text_response

def constraint_setter(input_prompts: str):
    response = client.responses.create(
        model="gpt-4o",
        input=input_prompts,
        tools=[{
            "type": "file_search",
            "vector_store_ids": ["vs_6820650b99c88191beb65d5a04c01bbd"]
        }]
    )
    text_response = None
    for output in response.output:
        if output.type == "message":
            for content_part in output.content:
                if content_part.type == "output_text":
                    text_response = content_part.text
                    break  # found it, no need to continue
        if text_response:
            break
    if text_response:
        print("\nExtracted Text Response:\n")
        print(text_response)
    else:
        print("constraint setter agent error!")
    return text_response


def parse_asset_selector_gptres(text_response: str):
    # Pattern to match JSON content between triple backticks
    json_pattern = r'```(?:json)?\s*([\s\S]*?)```'
    
    # Find all matches
    matches = re.findall(json_pattern, text_response)
    
    if matches:
        # Get the first JSON match
        json_str = matches[0].strip()
        try:
            # Parse the JSON to ensure it's valid
            json_object = json.loads(json_str)
            return json_object
        except json.JSONDecodeError:
            try:
                # If failed, try replacing single quotes with double quotes
                # First, escape any existing double quotes
                json_str = json_str.replace('"', '\\"')
                # Then replace single quotes with double quotes
                json_str = json_str.replace("'", '"')
                # Finally, try to parse again
                json_object = json.loads(json_str)
                return json_object
            except json.JSONDecodeError:
                return "Invalid JSON format"
    else:
        return "No JSON found in response"


def parse_constraint_setter_gptres(text_response: str):
    # Pattern to match JSON content between triple backticks
    json_pattern = r'```(?:json)?\s*([\s\S]*?)```'
    
    # Find all matches
    matches = re.findall(json_pattern, text_response)
    
    if matches:
        # Get the first JSON match
        json_str = matches[0].strip()
        
        # Convert Python tuples to JSON arrays
        # Replace ("key", "value") with ["key", "value"]
        #json_str = re.sub(r'\(([^)]*?)\)', r'[\1]', json_str)
        
        # Clean up common JSON syntax issues
        
        # 1. Fix missing commas between arrays in arrays
        json_str = re.sub(r'\](\s*)\[', r'],\1[', json_str)
        
        # 3. Fix any extra closing braces
        brace_count = json_str.count('{') - json_str.count('}')
        if brace_count > 0:
            # Add missing closing braces
            json_str += '}' * brace_count
        elif brace_count < 0:
            # Remove extra closing braces
            json_str = json_str[:json_str.rfind('}')] + json_str[json_str.rfind('}')+1:]

        try:
            # Parse the JSON to ensure it's valid
            json_object = json.loads(json_str)
            return json_object
        except json.JSONDecodeError as e:
            return f"Invalid JSON format: {e}"
    else:
        return "No JSON found in response"

def convert_constraints(constraints_dict: dict):
    # loopup_dict
    with open(r'E:\Scientific Research\conlan\data\lookup_dict.json', 'r') as f:
        lookup_dict = json.load(f)

    object_list = []
    # Process each constraint and find corresponding object size
    for constraint_key, constraint_values in constraints_dict.items():
        # Remove the index suffix (-0, -1, etc.)
        base_name = re.sub(r'-\d+$', '', constraint_key)
        
        # Find the object in the lookup dictionary
        if base_name in lookup_dict:
            obj = lookup_dict[base_name]
            length, width = obj["size"][0], obj["size"][2]
            object_list.append((constraint_key, (length, width)))

        else:
            print(f"Object: {base_name} not found in JSON data")
            print(f"  Constraints: {constraint_values}")
    return object_list


