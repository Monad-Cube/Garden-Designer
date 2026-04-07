from sys_msg_en_conlan import *
from base import *
from pcg import *
from edit import *
#import debugpy; debugpy.listen(('127.0.0.10', 57000)); debugpy.wait_for_client()
class MyThread(threading.Thread):
    def __init__(self, func, args=()):
        super(MyThread, self).__init__()
        self.func = func
        self.args = args
        self.result = None

    def run(self):
        # gets executed when the thread starts.
        self.result = self.func(*self.args)

    def get_result(self):
        try:
            return self.result
        except Exception:
            return None


class UnionFind:  # each element is a tuple (x, y)
    def __init__(self, elements):
        self.elements = elements
        self.father = {}
        for element in elements:
            self.father[element] = element

    def find(self, element):
        if self.father[element] == element:
            return element
        self.father[element] = self.find(self.father[element])
        return self.father[element]

    def union(self, element1, element2):
        father1, father2 = self.find(element1), self.find(element2)
        if father1 != father2:
            self.father[father1] = father2

    def get_components(self):
        components = {}
        for element in self.elements:
            father = self.find(element)
            if father not in components:
                components[father] = []
            components[father].append(element)
        return components


@retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
def completion_with_backoff(**kwargs):
    return client.chat.completions.create(**kwargs)


def modify_parameter(fname: str, key_vals: dict):
    lines = []
    with open(fname, "r") as f:
        lines = f.readlines()
    for i in range(len(lines)):
        comps = lines[i].split()
        if len(comps) == 0:
            continue
        line_start = comps[0]
        for key in key_vals:
            if line_start == key:
                lines[i] = key + " = " + str(key_vals[key]) + "\n"
                break
    with open(fname, "w") as f:
        f.writelines(lines)


def unit_corner_dis(unit: Tuple[int, int], corner: Tuple[int, int]) -> int:
    """
    
    """
    unit_corners = [(unit[0], unit[1]), (unit[0] + 1, unit[1]), (unit[0], unit[1] + 1), (unit[0] + 1, unit[1] + 1)]
    dists = [abs(unit_corner[0] - corner[0]) + abs(unit_corner[1] - corner[1]) for unit_corner in unit_corners]
    return min(dists)


def find_connected_components(grid: List[List[int]], split_edges: dict, n: int) -> List[list]:
    def is_valid(x, y):
        # Check if the coordinates are within the grid boundaries
        return 0 <= x < W and 0 <= y < H

    def dfs(x, y, component_id):
        stack = [(x, y)]
        component = []

        dx_dys = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        while stack:
            current_x, current_y = stack.pop()
            if component_labels[current_x][current_y] != -1:
                continue

            component_labels[current_x][current_y] = component_id
            component.append((current_x, current_y))

            for dx, dy in dx_dys:
                new_x, new_y = current_x + dx, current_y + dy
                if dx == 0 and dy == 1:
                    split_edge = ((current_x, current_y + 1), (current_x + 1, current_y + 1))
                elif dx == 0 and dy == -1:
                    split_edge = ((current_x, current_y), (current_x + 1, current_y))
                elif dx == 1 and dy == 0:
                    split_edge = ((current_x + 1, current_y), (current_x + 1, current_y + 1))
                elif dx == -1 and dy == 0:
                    split_edge = ((current_x, current_y), (current_x, current_y + 1))
                # not overstep; adjacent grids have same terrain type; 
                if (
                    is_valid(new_x, new_y)
                    and grid[new_x][new_y] == grid[x][y] # why not current_x/y
                    and ((split_edge not in split_edges) or split_edges[split_edge] == 0)
                ):
                    stack.append((new_x, new_y))

        return component

    ### component_labels, W*H Grid, [[-1, -1, ...H], [], [], ...W], 
    # used to judge whether current (x,y) has been append to components
    ### components = [[], [], [], [], []], list contains componets of different type,
    # each component contains the (x,y) of the whole connected components
    component_labels = [[-1 for _ in range(H)] for _ in range(W)]
    components = [[] for _ in range(n)]

    component_id = 0
    # W*H Grid
    for x in range(W):
        for y in range(H):
            if component_labels[x][y] == -1:
                # for (x,y), find the neighbors of same terrain type
                # and iteratively for the whole connected area, 
                # and assign the component id (area compoent's order)
                component = dfs(x, y, component_id) 
                components[grid[x][y]].append(component)
                component_id += 1

    return components


def get_boundary(grid: List[List[int]], get: bool = False) -> Tuple[bool, List[Tuple[int, int]]]:
    """
    numbers other than 0(unused) is considered as content
    return the boundary of the content, represented by a list of coordinates of corners
    if there are no or more than one connected components, return False
    """
    new_grid = deepcopy(grid)
    for i in range(W):
        for j in range(H):
            if new_grid[i][j] != 0:
                new_grid[i][j] = 1
    components = find_connected_components(new_grid, {}, 2)
    if len(components[1]) != 1:
        return False, []
    if not get:
        return True, []

    boundary_edges = []
    for i in range(W):
        for j in range(H + 1):
            if j == 0:
                if new_grid[i][j] == 1:
                    boundary_edges.append([(i, j), (i + 1, j)])
            elif j == H:
                if new_grid[i][j - 1] == 1:
                    boundary_edges.append([(i, j), (i + 1, j)])
            elif new_grid[i][j - 1] != new_grid[i][j]:
                boundary_edges.append([(i, j), (i + 1, j)])
    for i in range(W + 1):
        for j in range(H):
            if i == 0:
                if new_grid[i][j] == 1:
                    boundary_edges.append([(i, j), (i, j + 1)])
            elif i == W:
                if new_grid[i - 1][j] == 1:
                    boundary_edges.append([(i, j), (i, j + 1)])
            elif new_grid[i - 1][j] != new_grid[i][j]:
                boundary_edges.append([(i, j), (i, j + 1)])

    ordered_boundary_corners = [boundary_edges[0][0]]
    while True:
        corner = ordered_boundary_corners[-1]
        for edge in boundary_edges:
            if edge[0] == corner:
                ordered_boundary_corners.append(edge[1])
                boundary_edges.remove(edge)
                break
            elif edge[1] == corner:
                ordered_boundary_corners.append(edge[0])
                boundary_edges.remove(edge)
                break
        if ordered_boundary_corners[-1] == ordered_boundary_corners[0]:
            break
    return True, ordered_boundary_corners[:-1]

def get_center(component: List[Tuple[int, int]]) -> Tuple[float, float]:
    x_coords = [x for x,y in component]
    y_coords = [y for x,y in component]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    return (min_x + max_x) / 2, (min_y + max_y) / 2


def query(system_messages: List[dict], text: str) -> str:
    messages = system_messages
    messages.append({"role": "user", "content": text})
    try:
        chat_completion = completion_with_backoff(
            model="gpt-4o-mini", #"gpt-4o", "gpt-5"
            messages=messages,
            #model="gpt-3.5-turbo",
            #messages=messages,
        )
        res = chat_completion.choices[0].message.content
        return res
    except Exception as e:
        print("gpt exception", e)
        return ""


def form_sys_messages(sys_prompt, sample_inputs, sample_outputs):
    sys_messages = [{"role": "system", "content": sys_prompt}]
    for i in range(len(sample_inputs)):
        sys_messages.append({"role": "user", "content": sample_inputs[i]})
        sys_messages.append({"role": "assistant", "content": sample_outputs[i]})
    return sys_messages


def parse_terrain_gptres(text: str) -> Tuple[dict, str]:
    # default parameters
    dic = {}
    keys = ["terrain_exist", "terrain_region_num", "terrain_region_area", "terrain_region_single_area"]
    for key in keys:
        dic[key] = [-1 for _ in range(5)]
    dic["max_hill_height"] = -1

    # parse
    try:
        json_data = json.loads(text)
        data, fb = json_data["data"], json_data["feedback"]
        assert len(data) == 5
        lis = data[0]
        assert len(lis) == 5
        for i in range(5):
            assert lis[i] == 0 or lis[i] == 1 or lis[i] == -1
            dic["terrain_exist"][i] = lis[i]
        for j in range(1, 4):
            lis = data[j]
            assert len(lis) == 5
            for i in range(5):
                assert len(lis[i]) == 2
                assert lis[i][0] == -1 or lis[i][0] >= 0
                assert lis[i][1] == -1 or lis[i][1] >= 0
                if j == 1:
                    dic["terrain_region_num"][i] = lis[i]
                elif j == 2:
                    dic["terrain_region_area"][i] = lis[i]
                elif j == 3:
                    dic["terrain_region_single_area"][i] = lis[i]
        assert data[4] == -1 or data[4] > 0
        dic["max_hill_height"] = data[4]
        return dic, fb
    except Exception as e:
        print("parse exception:", e)
        return dic, "GPT response parsing error"


def parse_inf_gptres(text: str) -> Tuple[dict, str]:
    # default parameters
    dic = {}
    keys = ["inf_entrance_num", "inf_keyunit_num", "inf_mainroad_width", "inf_road_complexity"]
    for key in keys:
        dic[key] = -1
    # parse
    try:
        json_data = json.loads(text)
        data, fb = json_data["data"], json_data["feedback"]
        assert len(data) == 4
        for i in range(4):
            lis = data[i]
            assert len(lis) == 2
            assert lis[0] == -1 or lis[0] >= 0
            assert lis[1] == -1 or lis[1] >= 0
            dic[keys[i]] = lis
        return dic, fb
    except Exception as e:
        print("parse exception:", e)
        return dic, "GPT response parsing error"


def parse_attribute_gptres(text: str) -> Tuple[dict, str]:
    # default parameters
    dic = {}
    keys = ["attribute_exist", "attribute_region_num", "attribute_region_area", "attribute_region_single_area"]
    for key in keys:
        dic[key] = [-1 for _ in range(5)]

    # parse
    try:
        json_data = json.loads(text)
        data, fb = json_data["data"], json_data["feedback"]
        assert len(data) == 4
        lis = data[0]
        for i in range(5):
            assert lis[i] == 0 or lis[i] == 1 or lis[i] == -1
            dic["attribute_exist"][i] = lis[i]
        for j in range(1, 4):
            lis = data[j]
            assert len(lis) == 5
            for i in range(5):
                assert len(lis[i]) == 2
                assert lis[i][0] == -1 or lis[i][0] >= 0
                assert lis[i][1] == -1 or lis[i][1] >= 0
                if j == 1:
                    dic["attribute_region_num"][i] = lis[i]
                elif j == 2:
                    dic["attribute_region_area"][i] = lis[i]
                elif j == 3:
                    dic["attribute_region_single_area"][i] = lis[i]
        return dic, fb
    except Exception as e:
        print("parse exception:", e)
        return dic, "GPT response parsing error"

def default_parameters() -> dict:
    terrain_parameters, terrain_feedback = parse_terrain_gptres("")
    inf_parameters, inf_feedback = parse_inf_gptres("")
    attribute_parameters, attribute_feedback = parse_attribute_gptres("")

    parameters = {}
    parameters.update(terrain_parameters)
    parameters.update(inf_parameters)
    parameters.update(attribute_parameters)
    return parameters


def visualize(
    grid: List[List[int]],
    contents: List[List[int]],
    entrances: List[Tuple[int, int]],
    primary_edges: List[Tuple[Tuple[int, int], Tuple[int, int]]],
    secondary_edges: List[Tuple[Tuple[int, int], Tuple[int, int]]],
    keyunits: List[Tuple[int, int]],
    colors: List[Tuple[float, float, float]],
    grid_labelset: List[str],
    content_markerset: List[str],
    content_labelset: List[str],
    name: str,
) -> None:
    """
    grid: W * H, units
    contents: W * H, units
    entrances: corners
    primary_edges: edges
    secondary_edges: edges
    keyunits: units

    grid[i][j]: 0 ~ n-1
    contents[i][j]: 0 ~ m-1. 0: draw nothing; 1: draw a line; 2: draw a cross; 3: draw a triangle; 4: draw a square
    colors[i]: (r,g,b), 0<=r,g,b<=1
    plot the grid using matplotlib, each grid[i][j] is a square with color colors[grid[i][j]]
    plot the primary_edges with color red, and secondary_edges with color blue
    plot the entrances with color purple
    surround the edges of keyunits with color black
    """
    fig, ax = plt.subplots(figsize=(17, 6))
    sum_contents = sum([sum(i) for i in contents])
    for i in range(W):
        for j in range(H):
            ax.add_patch(plt.Rectangle((i, j), 1, 1, color=colors[grid[i][j]], alpha=0.5, linewidth=1))
            if contents[i][j] >= 1 and grid[i][j] > 0:
                ax.plot(i + 0.5, j + 0.5, color="black", marker=content_markerset[contents[i][j]], markersize=8)
    for edge in primary_edges:
        ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color="red", linewidth=2)
    for edge in secondary_edges:
        ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color="blue", linewidth=1)
    for entrance in entrances:
        ax.plot(entrance[0], entrance[1], color="purple", marker="o", markersize=10)
    for keyunit in keyunits:
        ax.add_patch(plt.Rectangle((keyunit[0], keyunit[1]), 1, 1, color="black", fill=False, linewidth=2))

    handles1 = [
        matplotlib.patches.Patch(color=colors[i], label=grid_labelset[i])
        for i in range(len(grid_labelset))
        if i < len(colors)
    ]
    handles2 = [
        matplotlib.lines.Line2D([], [], color="red", label="Primary Road"),
        matplotlib.lines.Line2D([], [], color="blue", label="Secondary Road"),
        matplotlib.lines.Line2D([], [], color="purple", marker="o", label="Entrance", linestyle="None"),
        plt.Rectangle((0, 0), 1, 1, color="black", fill=False, label="Point of Interest"),
    ]
    handles3 = [
        matplotlib.lines.Line2D(
            [], [], color="black", marker=content_markerset[i], label=content_labelset[i], linestyle="None"
        )
        for i in range(len(content_labelset))
        if i < len(content_markerset)
    ]
    legend1 = plt.legend(handles=handles1, handlelength=0.8, fontsize=20, bbox_to_anchor=(0.96, 0.98))
    ax.add_artist(legend1)
    if len(primary_edges) + len(secondary_edges) > 0:
        legend2 = plt.legend(handles=handles2, handlelength=0.8, fontsize=20, bbox_to_anchor=(0.96, 0.46))
        ax.add_artist(legend2)
    if sum_contents > 0:
        legend3 = plt.legend(handles=handles3, handlelength=0.8, fontsize=20, bbox_to_anchor=(1.75, 0.98))
        ax.add_artist(legend3)
    # ax.legend(
    #     handles=handles,
    #     bbox_to_anchor=(0.96, 1),
    #     handlelength=0.8,
    #     fontsize=11.8
    # )
    plt.axis("scaled")
    plt.axis("off")
    plt.savefig("visuals/" + name + ".png")
    plt.close()
    plt.clf()


def visualize_height_map(height_map: List[List[float]], width: int, height: int, name: str) -> None:
    fig, ax = plt.subplots(constrained_layout=True)
    for i in range(width):
        for j in range(height):
            ax.add_patch(
                plt.Rectangle((i, j), 1, 1, color=(height_map[i][j], height_map[i][j], height_map[i][j]), linewidth=0)
            )
    plt.axis("scaled")
    plt.savefig("visuals/" + name + ".png")
    plt.close()
    plt.clf()


def visualize_continuous(
    corners: List[Tuple[int, int]],
    points: List[Tuple[float, float]],
    edges: List[Tuple[List[Tuple[int, int]], List[int]]],
    areas: List[Tuple[int, List[int]]],
    colors: List[Tuple[float, float, float]],
    name: str,
) -> None:
    fig, ax = plt.subplots(constrained_layout=True)
    # print(areas)
    for area in areas:
        combined_type, point_idxs = area
        if len(point_idxs) < 3:
            continue
        ax.add_patch(
            plt.Polygon(
                [points[idx] for idx in point_idxs],
                color=colors[combined_type],
                linewidth=0,
            )
        )
    for i in range(len(edges)):
        edge_idx_list, edge_info_list = edges[i]
        for edge_idx in edge_idx_list:
            p1, p2 = points[edge_idx[0]], points[edge_idx[1]]
            if 7 in edge_info_list:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(238/255, 179/255, 194/255), linewidth=1) # pink
            elif 8 in edge_info_list:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(244/255, 248/255, 211/255), linewidth=1) # yellow
            elif 9 in edge_info_list:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(166/255, 214/255, 214/255), linewidth=1) # blue
            elif 10 in edge_info_list:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(142/255, 125/255, 190/255), linewidth=1) # purple
            else:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=(0, 0, 0), linewidth=1) # black

            
    for corner in corners:
        ax.plot(corner[0], corner[1], color="purple", marker="o", markersize=1)

    plt.axis("off")
    plt.savefig("visuals/" + name + ".png")
    plt.close()
    plt.clf()


def randominit(n: int, splits: int) -> List[List[int]]:
    """
    randomly generate a grid
    partition the grid into 5*5 blocks
    give all units in each block a random number from 0 to n-1
    """
    # split is 5, H is 15, W is 20 in default
    # NOTE possible bug: grid[x][y], optimize later
    grid = [[0 for _ in range(H)] for _ in range(W)]
    assert W % splits == 0 and H % splits == 0
    blockw, blockh = W // splits, H // splits
    # loop through splits
    for i in range(splits):
        for j in range(splits):
            randnum = random.randint(0, n - 1)
            # loop inner block
            for k in range(blockw):
                for l in range(blockh):
                    grid[i * blockw + k][j * blockh + l] = randnum # grid[x][y]
    # NOTE why init twice? for the boundary process
    for i in range(splits - 1):
        for j in range(splits - 1):
            randnum = random.randint(0, n - 1)
            for k in range(blockw):
                for l in range(blockh):
                    grid[i * blockw + blockw // 2 + k][j * blockh + blockh // 2 + l] = randnum
    return grid


def terrain_fitness(grid: List[List[int]], parameters: dict, terrain, infrastructure) -> float:
    """
    0 for unused, 1 for lake, 2 for land, 3 for ground, 4 for hill, (5 for border)
    """
    connected, _ = get_boundary(grid)
    if not connected:
        return 0

    components = find_connected_components(grid, {}, 5)
    terrain_exist, terrain_region_num, terrain_region_area, terrain_region_single_area = (
        deepcopy(parameters["terrain_exist"]),
        deepcopy(parameters["terrain_region_num"]),
        deepcopy(parameters["terrain_region_area"]),
        deepcopy(parameters["terrain_region_single_area"]),
    )
    loss = 1
    for tp in range(5):
        exist, region_num, region_area, region_single_area = (
            terrain_exist[tp],
            terrain_region_num[tp],
            terrain_region_area[tp],
            terrain_region_single_area[tp],
        )

        # exist
        real_region_num = len(components[tp])
        if exist == -1:
            exist = 1
        if exist == 0 and real_region_num > 0:
            loss += 20
            continue
        elif exist == 1 and real_region_num == 0:
            loss += 20
            continue

        if real_region_num == 0:
            continue

        # region_num
        if region_num == -1:
            region_num = (-1, -1)
        lo, hi = region_num
        if tp == 1 and hi == -1:
            hi = 3 if lo == -1 else max(lo, 3)
        elif tp == 2 and hi == -1:
            hi = 4 if lo == -1 else max(lo, 4)
        elif tp == 3 and hi == -1:
            hi = 3 if lo == -1 else max(lo, 3)
        elif tp == 4 and hi == -1:
            hi = 3 if lo == -1 else max(lo, 3)
        if lo != -1 and real_region_num < lo:
            loss += min(lo - real_region_num, 5)
        if hi != -1 and real_region_num > hi:
            loss += min(real_region_num - hi, 5)

        # region_area
        if region_area == -1:
            region_area = (-1, -1)
        real_region_area = 0
        for component in components[tp]:
            real_region_area += len(component)
        real_region_area /= W * H
        lo, hi = region_area
        if tp == 0:
            if hi == -1:
                hi = 0.2 if lo == -1 else max(lo, 0.2)
            if lo == -1:
                lo = 0.05 if hi == -1 else min(hi, 0.05)
        elif tp == 1:
            if hi == -1:
                hi = 0.6 if lo == -1 else max(lo, 0.6)
            if lo == -1:
                lo = 0.2 if hi == -1 else min(hi, 0.2)
        elif tp == 2:
            if hi == -1:
                hi = 0.7 if lo == -1 else max(lo, 0.7)
            if lo == -1:
                lo = 0.3 if hi == -1 else min(hi, 0.3)
        elif tp == 3:
            if hi == -1:
                hi = 0.4 if lo == -1 else max(lo, 0.4)
            if lo == -1:
                lo = 0.1 if hi == -1 else min(hi, 0.1)
        elif tp == 4:
            if hi == -1:
                hi = 0.35 if lo == -1 else max(lo, 0.35)
            if lo == -1:
                lo = 0.15 if hi == -1 else min(hi, 0.15)
        if lo != -1 and real_region_area < lo:
            loss += min((lo - real_region_area) * 100, 5)
        if hi != -1 and real_region_area > hi:
            loss += min((real_region_area - hi) * 100, 5)

        # region_single_area
        if region_single_area == -1:
            region_single_area = (-1, -1)
        real_region_single_area = []
        for component in components[tp]:
            real_region_single_area.append(len(component) / W / H)
        lo, hi = region_single_area
        if tp == 1 and lo == -1:
            lo = 0.02 if hi == -1 else min(hi, 0.02)
        elif tp == 2 and lo == -1:
            lo = 0.02 if hi == -1 else min(hi, 0.02)
        elif tp == 3 and lo == -1:
            lo = 0.01 if hi == -1 else min(hi, 0.01)
        elif tp == 4 and lo == -1:
            lo = 0.02 if hi == -1 else min(hi, 0.02)
        local_loss = 0
        for area in real_region_single_area:
            if lo != -1 and area < lo:
                local_loss += min((lo - area) * 100, 2)
            if hi != -1 and area > hi:
                local_loss += min((area - hi) * 100, 2)
        loss += min(local_loss, 5)

    # adjacent relations
    dx_dys = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    adjacent_types = [[] for _ in range(5)]
    land_in_lake_count = 0
    for tp in range(5):
        for component in components[tp]:
            types_count = [0 for _ in range(6)]
            for x, y in component:
                for dx, dy in dx_dys:
                    new_x, new_y = x + dx, y + dy
                    if 0 <= new_x < W and 0 <= new_y < H:
                        if (new_x, new_y) not in component:
                            types_count[grid[new_x][new_y]] += 1
                    else:
                        types_count[-1] += 1
            adjacent_types[tp].append(types_count)
            if tp == 0:
                if types_count[-1] == 0:  # unused must be adjacent to border
                    loss += min(types_count[1] * 0.8, 15)
            elif tp == 1:
                if types_count[4] > 0:  # lake cannot be adjacent to hill
                    loss += min(types_count[4] * 0.3, 5)
                if types_count[0] + types_count[-1] > 0:  # lake cannot be adjacent to border or unused
                    loss += min((types_count[0] + types_count[-1]) * 0.2, 3)
            elif tp == 2 or tp == 3:
                if (
                    types_count[0] + types_count[-1] + types_count[2] + types_count[3] + types_count[4] == 0
                    and len(component) / (W * H) < 0.02
                ):
                    land_in_lake_count += 1
            elif tp == 4:
                if types_count[0] + types_count[-1] > 0:  # hill cannot be adjacent to border or unused
                    loss += min((types_count[0] + types_count[-1]) * 0.3, 5)
    # lake count penalty
    lake_count = 0
    for component in components[1]:
        if len(component) / (W * H) > 0.08:
            lake_count += 1
    if land_in_lake_count < lake_count:
        loss += min((lake_count - land_in_lake_count) * 2, 5)

    fitness = 100 / loss
    return fitness


def terrain_fitness_with_loc(grid: List[List[int]], parameters: dict, terrain, infrastructure, generation: int) -> float:
    """
    0 for unused, 1 for lake, 2 for land, 3 for ground, 4 for hill, (5 for border)
    Add location control to the fitness function
    Variable:
    - conmponet[i][j] is tuple (x, y) for the location of the j-th component of type i    
    """
    connected, _ = get_boundary(grid)
    if not connected:
        return 0

    components = find_connected_components(grid, {}, 5)
    terrain_exist, terrain_region_num, terrain_region_area, terrain_region_single_area = (
        deepcopy(parameters["terrain_exist"]),
        deepcopy(parameters["terrain_region_num"]),
        deepcopy(parameters["terrain_region_area"]),
        deepcopy(parameters["terrain_region_single_area"]),
    )
    loss = 1
    for tp in range(5):
        exist, region_num, region_area, region_single_area = (
            terrain_exist[tp],
            terrain_region_num[tp],
            terrain_region_area[tp],
            terrain_region_single_area[tp],
        )
        # NOTE exist
        real_region_num = len(components[tp])
        if tp in [1,2,3] : exist = 1
        if exist == -1:
            exist = 1
        if exist == 0 and real_region_num > 0:
            loss += 20
            continue
        elif exist == 1 and real_region_num == 0:
            loss += 20
            continue

        if real_region_num == 0:
            continue

        # NOTE region_num
        if region_num == -1:
            region_num = (-1, -1)
        lo, hi = region_num
        if tp == 1 and hi == -1:
            hi = 1 if lo == -1 else max(lo, 3)
        elif tp == 2 and hi == -1:
            hi = 2 if lo == -1 else max(lo, 2)
        elif tp == 3 and hi == -1:
            hi = 3 if lo == -1 else max(lo, 3)
        elif tp == 4 and hi == -1:
            hi = 3 if lo == -1 else max(lo, 3)
        if lo != -1 and real_region_num < lo:
            loss += min(lo - real_region_num, 5)
        if hi != -1 and real_region_num > hi:
            loss += min(real_region_num - hi, 5)

        # NOTE region_area
        if region_area == -1:
            region_area = (-1, -1)
        real_region_area = 0
        # not the correct estimation, but it is efficient as the area is difficult to caculate
        for component in components[tp]:
            real_region_area += len(component)
        real_region_area /= W * H
        lo, hi = region_area
        if tp == 0: # default [0.05, 0.2]
            if hi == -1:
                hi = 0.15 if lo == -1 else max(lo, 0.15)
            if lo == -1:
                lo = 0.05 if hi == -1 else min(hi, 0.05)
        elif tp == 1: # default [0.2, 0.4]
            if hi == -1:
                hi = 0.4 if lo == -1 else max(lo, 0.4)
            if lo == -1:
                lo = 0.2 if hi == -1 else min(hi, 0.2)
        elif tp == 2: # default [0.3, 0.7] -> change to [0.2, 0.4]
            if hi == -1:
                hi = 0.3 if lo == -1 else max(lo, 0.3)
            if lo == -1:
                lo = 0.1 if hi == -1 else min(hi, 0.1)
        elif tp == 3: # default [0.1, 0.4]
            if hi == -1:
                hi = 0.5 if lo == -1 else max(lo, 0.5)
            if lo == -1:
                lo = 0.2 if hi == -1 else min(hi, 0.2)
        elif tp == 4: # default [0.15, 0.35]
            if hi == -1:
                hi = 0 if lo == -1 else max(lo, 0)
            if lo == -1:
                lo = 0 if hi == -1 else min(hi, 0)
        if lo != -1 and real_region_area < lo:
            loss += min((lo - real_region_area) * 100, 5)
        if hi != -1 and real_region_area > hi:
            loss += min((real_region_area - hi) * 100, 5)

        # NOTE region_location
        for component in components[tp]:
            #control the center of the lake
            if tp == 1:
                # the center pos should around the center
                '''centerX, centerY = get_center(component)
                dx = abs(centerX - W / 2)
                dy = abs(centerY - H / 2)
                if dx > 2 or dy > 2 or dx + dy > 3:
                    loss += max(0.25*generation-10, 5)'''
                # area should include center pixel
                center_pixel_loss = 10
                centering_pixel_count = 0
                for x, y in component:
                    """
                    distance =np.linalg.norm(np.array((x, y)) - np.array((W/2, H/2)))
                    if distance < 1.6:
                        center_pixel_loss = 0
                        break"""
                    split_W = floor(W/3)
                    split_H = floor(H/3)
                    if ceil(W/2-split_W)<= x < floor(W/2+split_W) and ceil(H/2-split_H)<= y < floor(H/2+split_H):
                        centering_pixel_count +=1
                if centering_pixel_count >=18:
                    center_pixel_loss = 0
                loss += center_pixel_loss
        
        # NOTE region_single_area
        if region_single_area == -1:
            region_single_area = (-1, -1)
        real_region_single_area = []
        for component in components[tp]:
            real_region_single_area.append(len(component) / W / H)
        lo, hi = region_single_area
        # set default range for different terrain types
        if tp == 1 and lo == -1:
            lo = 0.02 if hi == -1 else min(hi, 0.02)
        elif tp == 2 and lo == -1:
            lo = 0.01 if hi == -1 else min(hi, 0.01)
        elif tp == 3 and lo == -1:
            lo = 0.03 if hi == -1 else min(hi, 0.03)
        elif tp == 4 and lo == -1:
            lo = 0.00 if hi == -1 else min(hi, 0.00)
        local_loss = 0
        for area in real_region_single_area:
            if lo != -1 and area < lo:
                local_loss += min((lo - area) * 100, 2)
            if hi != -1 and area > hi:
                local_loss += min((area - hi) * 100, 2)
        loss += min(local_loss, 5)

    # NOTE adjacent relations
    dx_dys = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    adjacent_types = [[] for _ in range(5)]
    land_in_lake_count = 0
    for tp in range(5):
        for component in components[tp]:
            # For each component, types_count is the count of each type of adjacent cells
            # 0-5 present unused, lake, land, ground, hill, border, -1 is also border
            types_count = [0 for _ in range(6)]
            for x, y in component:
                for dx, dy in dx_dys:
                    new_x, new_y = x + dx, y + dy
                    if 0 <= new_x < W and 0 <= new_y < H:
                        if (new_x, new_y) not in component:
                            types_count[grid[new_x][new_y]] += 1
                    else:
                        types_count[-1] += 1 # border
            adjacent_types[tp].append(types_count)
            if tp == 0:
                if types_count[-1] == 0:  # unused must be adjacent to border
                    loss += min(types_count[1] * 0.8, 15)
            elif tp == 1:
                #if types_count[4] > 0:  # lake cannot be adjacent to hill
                #    loss += min(types_count[4] * 0.3, 5)
                if types_count[0] + types_count[-1] > 0:  # lake cannot be adjacent to border or unused
                    loss += min((types_count[0] + types_count[-1]) * 0.2, 3)
            elif tp == 2 or tp == 3:
                if (
                    types_count[0] + types_count[-1] + types_count[2] + types_count[3] + types_count[4] == 0
                    and len(component) / (W * H) < 0.02
                ):
                    land_in_lake_count += 1
            elif tp == 4:
                if types_count[0] + types_count[-1] > 0:  # hill cannot be adjacent to border or unused
                    loss += min((types_count[0] + types_count[-1]) * 0.3, 5)
    # NOTE lake count penalty
    lake_count = 0
    for component in components[1]:
        if len(component) / (W * H) > 0.08:
            lake_count += 1
    if land_in_lake_count < lake_count:
        loss += min((lake_count - land_in_lake_count) * 2, 5)

    fitness = 100 / loss
    return fitness


def terrain_evo(grid: List[List[int]], parameters: dict, terrain, infrastructure) -> List[List[int]]:
    """
    0 for unused, 1 for lake, 2 for land, 3 for ground, 4 for hill, (5 for border)
    """
    components = find_connected_components(grid, {}, 5)
    terrain_exist = deepcopy(parameters["terrain_exist"])

    # check whether the terrain types in terrain_exit have existed
    for tp in range(5):
        exist = terrain_exist[tp]
        real_region_num = len(components[tp])
        if exist == 0 and real_region_num > 0:
            for component in components[tp]:
                for x, y in component:
                    grid[x][y] = 2
            return grid
        elif exist == 1 and real_region_num == 0:
            randw, randh = random.randint(2, 4), random.randint(2, 4)
            randx, randy = random.randint(0, W - randw), random.randint(0, H - randh)
            for i in range(randw):
                for j in range(randh):
                    grid[randx + i][randy + j] = tp
            return grid

    # check whether the unused grid is adjacent to border
    dx_dys = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for component in components[0]:
        types_count = [0 for _ in range(6)]
        for x, y in component:
            for dx, dy in dx_dys:
                new_x, new_y = x + dx, y + dy
                if 0 <= new_x < W and 0 <= new_y < H:
                    if (new_x, new_y) not in component:
                        types_count[grid[new_x][new_y]] += 1
                else:
                    types_count[-1] += 1
        if types_count[-1] == 0:  # current unused grid is not adjacent to border
            new_type = random.randint(1, 4)
            for x, y in component:
                grid[x][y] = new_type
            return grid

    return grid


def connect_try_single(
    c1: Tuple[int, int], c2: Tuple[int, int], new_uf: UnionFind, new_edge_used: dict, edge_scores: dict
) -> Tuple[float, UnionFind, dict]:
    """
    1.select the main road to connect two keypoints/entrance
    2.decrease the edge score of used edge in main road.
    """
    current_corner = c1
    last_direction = -1  # 0 for going left, 1 for top, 2 for right, 3 for bottom
    total_score = 0
    while new_uf.find(current_corner) != new_uf.find(c2):
        probablities = [1 for _ in range(4)]
        corners = [
            (current_corner[0] - 1, current_corner[1]),
            (current_corner[0], current_corner[1] + 1),
            (current_corner[0] + 1, current_corner[1]),
            (current_corner[0], current_corner[1] - 1),
        ]
        edges = [
            ((current_corner[0] - 1, current_corner[1]), current_corner),
            (current_corner, (current_corner[0], current_corner[1] + 1)),
            (current_corner, (current_corner[0] + 1, current_corner[1])),
            ((current_corner[0], current_corner[1] - 1), current_corner),
        ]
        for i in range(4):
            if edges[i] not in edge_scores:  # not valid
                probablities[i] = 0
            elif new_edge_used[edges[i]]:  # already used
                probablities[i] = 0
            else:
                probablities[i] *= pow(2, edge_scores[edges[i]])  # prefer to going along the edge with higher score
        if last_direction != -1:
            probablities[(last_direction + 2) % 4] = 0  # not going back
            probablities[last_direction] *= 2  # prefer to going straight
        # prefer to going towards c2
        dx, dy = c2[0] - current_corner[0], c2[1] - current_corner[1]
        if dx == 0:
            if dy > 0:
                probablities[1] *= 5
            else:
                probablities[3] *= 5
        elif dy == 0:
            if dx > 0:
                probablities[2] *= 5
            else:
                probablities[0] *= 5
        if dx > 0 and dy > 0:
            probablities[1] *= 3
            probablities[2] *= 3
        elif dx > 0 and dy < 0:
            probablities[2] *= 3
            probablities[3] *= 3
        elif dx < 0 and dy < 0:
            probablities[0] *= 3
            probablities[3] *= 3
        elif dx < 0 and dy > 0:
            probablities[0] *= 3
            probablities[1] *= 3
        if sum(probablities) < 1e-8:
            total_score = -1e9
            break
        probablities = [probablities[i] / sum(probablities) for i in range(4)]
        direction = random.choices([0, 1, 2, 3], probablities)[0] # select the first element after randomly selecting with weight
        new_edge_used[edges[direction]] = 1
        new_uf.union(current_corner, corners[direction])
        total_score += edge_scores[edges[direction]] - 1.5
        if last_direction != -1 and direction != last_direction:
            total_score -= 1.5
        last_direction = direction
        current_corner = corners[direction]

    return total_score, new_uf, new_edge_used


def connect_corners(
    c1: Tuple[int, int],
    c2: Tuple[int, int],
    uf: UnionFind,
    edge_used: dict,
    edge_scores: dict,
) -> Tuple[UnionFind, dict]:
    """
    select main road edge
    """
    if uf.find(c1) == uf.find(c2):
        return uf, edge_used
    max_score, max_uf, max_edge_used, results = -1e9, uf, edge_used, []
    # with Pool(NUM_WORKERS) as p:
    #     results = p.starmap(connect_try_single, [(c1,c2,deepcopy(uf),deepcopy(edge_used),edge_scores) for i in range(500)])
    # for result in results:
    for k in range(100):
        total_score, new_uf, new_edge_used = connect_try_single(c1, c2, deepcopy(uf), deepcopy(edge_used), edge_scores)
        total_score2, new_uf2, new_edge_used2 = connect_try_single(
            c2, c1, deepcopy(uf), deepcopy(edge_used), edge_scores
        )  # try the other direction
        if total_score > max_score:
            max_score, max_uf, max_edge_used = total_score, new_uf, new_edge_used
        if total_score2 > max_score:
            max_score, max_uf, max_edge_used = total_score2, new_uf2, new_edge_used2

    return max_uf, max_edge_used


def growth_split(component: List[Tuple[int, int]]) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    if len(component) < 2:
        return []
    labels = [0 for _ in range(len(component))]
    is_boundary = [0 for _ in range(len(component))]
    neighbors = [[] for _ in range(len(component))]
    dx_dys = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for i in range(len(component)):
        x, y = component[i]
        for dx, dy in dx_dys:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < W and 0 <= new_y < H:
                if (new_x, new_y) in component:
                    neighbors[i].append(component.index((new_x, new_y)))
                else:
                    is_boundary[i] = 1
            else:
                is_boundary[i] = 1
    # get farthest two units
    boundary_units = [component[i] for i in range(len(component)) if is_boundary[i] == 1]
    maxdis, maxunit1_idx, maxunit2_idx = -1, None, None
    for i in range(len(boundary_units)):
        for j in range(i + 1, len(boundary_units)):
            if (
                abs(boundary_units[i][0] - boundary_units[j][0]) + abs(boundary_units[i][1] - boundary_units[j][1])
                > maxdis
            ):
                maxdis = abs(boundary_units[i][0] - boundary_units[j][0]) + abs(
                    boundary_units[i][1] - boundary_units[j][1]
                )
                maxunit1_idx, maxunit2_idx = i, j
    # growth from the two units
    labels[maxunit1_idx] = 1
    labels[maxunit2_idx] = 2
    neighbors1, neighbors2 = set(n for n in neighbors[maxunit1_idx]), set(n for n in neighbors[maxunit2_idx])

    while True:
        # try to grow 1
        should_break = True
        while True:
            if len(neighbors1) == 0:
                break
            n1 = random.choice(list(neighbors1))
            if labels[n1] == 0:
                labels[n1] = 1
                for n in neighbors[n1]:
                    if labels[n] == 0:
                        neighbors1.add(n)
                neighbors1.remove(n1)
                should_break = False
                break
            neighbors1.remove(n1)
        # try to grow 2
        while True:
            if len(neighbors2) == 0:
                break
            n2 = random.choice(list(neighbors2))
            if labels[n2] == 0:
                labels[n2] = 2
                for n in neighbors[n2]:
                    if labels[n] == 0:
                        neighbors2.add(n)
                neighbors2.remove(n2)
                should_break = False
                break
            neighbors2.remove(n2)

        if should_break:
            break
    split_edges = []
    for i in range(W):
        for j in range(1, H):
            if (
                (i, j) in component
                and (i, j - 1) in component
                and labels[component.index((i, j))] != labels[component.index((i, j - 1))]
            ):
                split_edges.append(((i, j), (i + 1, j)))
    for i in range(1, W):
        for j in range(H):
            if (
                (i, j) in component
                and (i - 1, j) in component
                and labels[component.index((i, j))] != labels[component.index((i - 1, j))]
            ):
                split_edges.append(((i, j), (i, j + 1)))
    return split_edges


def attributes_fitness(
    grid: List[List[int]],
    parameters: dict,
    terrain: List[List[int]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[Tuple[int, int], Tuple[int, int]]], dict],
) -> float:
    """
    0 for no element, 1 for basic element, 2 for low-growing element, 3 for high-growing element, 4 for architectural element
    0: water, land, ground, etc; 1: grass, flower, lotus, etc; 2:shrub, flowerbed, rock, etc; 3: tree, etc; 4: building, etc
    """
    entrance_points, keyunits, edge_used = infrastructure
    _, boundary_points = get_boundary(terrain, True)
    boundary_edges = {}
    for i in range(len(boundary_points)):
        p1, p2 = boundary_points[i], boundary_points[(i + 1) % len(boundary_points)]
        if (p1, p2) in edge_used:
            boundary_edges[(p1, p2)] = 1
        else:
            boundary_edges[(p2, p1)] = 1
    components = find_connected_components(grid, boundary_edges, 5)
    attribute_exist, attribute_region_num, attribute_region_area, attribute_region_single_area = (
        deepcopy(parameters["attribute_exist"]),
        deepcopy(parameters["attribute_region_num"]),
        deepcopy(parameters["attribute_region_area"]),
        deepcopy(parameters["attribute_region_single_area"]),
    )
    loss = 1

    # remove unused
    for tp in range(5):
        iter = 0
        while iter < len(components[tp]):
            component = components[tp][iter]
            if terrain[component[0][0]][component[0][1]] == 0:
                components[tp].remove(component)
            else:
                iter += 1
    # basics
    for tp in range(5):
        exist, region_num, region_area, region_single_area = (
            attribute_exist[tp],
            attribute_region_num[tp],
            attribute_region_area[tp],
            attribute_region_single_area[tp],
        )

        # exist
        real_region_num = len(components[tp])
        if exist == -1:
            exist = 1
        if exist == 0 and real_region_num > 0:
            loss += 20
            continue
        elif exist == 1 and real_region_num == 0:
            loss += 20
            continue
        if real_region_num == 0:
            continue

        # region_num
        if region_num == -1:
            region_num = (-1, -1)
        lo, hi = region_num
        if tp == 0 and hi == -1:
            hi = 6 if lo == -1 else max(lo, 6)
        elif tp == 1 and hi == -1:
            hi = 4 if lo == -1 else max(lo, 4)
        elif tp == 2 and hi == -1:
            hi = 6 if lo == -1 else max(lo, 6)
        elif tp == 3 and hi == -1:
            hi = 6 if lo == -1 else max(lo, 6)
        elif tp == 4 and hi == -1:
            hi = 6 if lo == -1 else max(lo, 6)
        if lo != -1 and real_region_num < lo:
            loss += min(lo - real_region_num, 5)
        if hi != -1 and real_region_num > hi:
            loss += min(real_region_num - hi, 5)

        # region_area
        if region_area == -1:
            region_area = (-1, -1)
        real_region_area = 0
        for component in components[tp]:
            real_region_area += len(component)
        real_region_area /= W * H
        lo, hi = region_area
        if tp == 0:
            if hi == -1:
                hi = 0.4 if lo == -1 else max(lo, 0.4)
            if lo == -1:
                lo = 0.15 if hi == -1 else min(hi, 0.15)
        elif tp == 1:
            if hi == -1:
                hi = 0.4 if lo == -1 else max(lo, 0.4)
            if lo == -1:
                lo = 0.05 if hi == -1 else min(hi, 0.05)
        elif tp == 2:
            if hi == -1:
                hi = 0.3 if lo == -1 else max(lo, 0.3)
        elif tp == 3:
            if hi == -1:
                hi = 0.4 if lo == -1 else max(lo, 0.4)
            elif lo == -1:
                lo = 0.2 if hi == -1 else min(hi, 0.2)
        elif tp == 4:
            if hi == -1:
                hi = 0.25 if lo == -1 else max(lo, 0.25)
            elif lo == -1:
                lo = 0.15 if hi == -1 else min(hi, 0.15)
        if lo != -1 and real_region_area < lo:
            loss += min((lo - real_region_area) * 100, 5)
        if hi != -1 and real_region_area > hi:
            loss += min((real_region_area - hi) * 100, 5)

        # region_single_area
        if region_single_area == -1:
            region_single_area = (-1, -1)
        real_region_single_area = []
        for component in components[tp]:
            real_region_single_area.append(len(component) / W / H)
        lo, hi = region_single_area
        if tp == 0 and lo == -1:
            lo = 0.005 if hi == -1 else min(hi, 0.005)
        elif tp == 1 and lo == -1:
            lo = 0.005 if hi == -1 else min(hi, 0.005)
        elif tp == 3 and lo == -1:
            lo = 0.03 if hi == -1 else min(hi, 0.03)
        elif tp == 4 and hi == -1:
            hi = 0.05 if lo == -1 else max(lo, 0.05)
        local_loss = 0
        for area in real_region_single_area:
            if lo != -1 and area < lo:
                local_loss += min((lo - area) * 100, 2)
            if hi != -1 and area > hi:
                local_loss += min((area - hi) * 100, 2)
        loss += min(local_loss, 5)

    # relation with terrain
    splited_components = find_connected_components(grid, edge_used, 5)
    for tp in range(5):
        local_loss = 0
        for component in splited_components[tp]:
            terrain_tp = terrain[component[0][0]][component[0][1]]
            if terrain_tp == 0:  # unused
                continue
            ratio = len(component) / (W * H)
            if tp == 0:  # no element
                if ratio < 0.005:
                    local_loss += (0.005 - ratio) * 100
                if terrain_tp == 4:  # hill
                    local_loss += min(ratio * 50, 2)
                if terrain_tp > 1 and ratio > 0.01:
                    local_loss += min((ratio - 0.01) * 200, 8)
                elif terrain_tp == 1 and ratio < 0.02:
                    local_loss += min((0.02 - ratio) * 100, 3)
            elif tp == 1:  # basic element
                if ratio < 0.005:
                    local_loss += (0.005 - ratio) * 100
                if terrain_tp == 1 and ratio > 0.025:  # lake
                    local_loss += min((ratio - 0.025) * 100, 3)
                elif terrain_tp == 3:  # ground
                    local_loss += min(ratio * 100, 3)
                elif terrain_tp == 4:  # hill
                    local_loss += min(ratio * 50, 2)
            elif tp == 2:  # low-growing element
                if ratio < 0.005:
                    local_loss += (0.005 - ratio) * 100
                if terrain_tp == 1 and len(component) / (W * H) > 0.02:  # too much low-growing element in lake
                    local_loss += min((ratio - 0.02) * 100, 3)
                elif terrain_tp == 4:  # hill
                    local_loss += min(ratio * 30, 1)
            elif tp == 3:  # high-growing element
                if ratio < 0.005:
                    local_loss += (0.005 - ratio) * 100
                if terrain_tp == 1:  # decrease the occupation of tree
                    local_loss += min(ratio * 100, 5)  
                if terrain_tp == 1:  # lake
                    local_loss += min(ratio * 200, 5)
            elif tp == 4:
                if ratio > 0.02:
                    local_loss += min((ratio - 0.02) * 100, 3)
                if terrain_tp == 1:  # lake
                    local_loss += min(ratio * 200, 5)
        loss += min(local_loss, 15)

    # relation with keyunits
    near_4 = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    near_8 = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (-1, 1), (1, -1)]
    for keyunit in keyunits:
        kx, ky = keyunit
        keyunit_type = grid[kx][ky]
        if keyunit_type == 0:
            loss += 10 / len(keyunits)
        elif keyunit_type != 4:
            local_loss = 4 / len(keyunits)
            for dx, dy in near_8:
                new_x, new_y = kx + dx, ky + dy
                if 0 <= new_x < W and 0 <= new_y < H:
                    if grid[new_x][new_y] == 4:
                        local_loss -= 1 / len(keyunits)
            loss += max(1 / len(keyunits), local_loss)
        types_set = set([keyunit_type])
        for dx, dy in near_4:
            new_x, new_y = kx + dx, ky + dy
            if 0 <= new_x < W and 0 <= new_y < H:
                types_set.add(grid[new_x][new_y])
        if len(types_set) == 1:
            loss += 10 / len(keyunits)
        elif len(types_set) == 2:
            loss += 2 / len(keyunits)

    loss = max(1, loss)
    return 100 / loss


def attributes_evo(
    grid: List[List[int]],
    parameters: dict,
    terrain: List[List[int]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[Tuple[int, int], Tuple[int, int]]], dict],
) -> List[List[int]]:
    entrance_points, keyunits, edge_used = infrastructure
    splited_terrain_components = find_connected_components(terrain, edge_used, 5)
    components_count = 0
    for i in range(1, 5):
        components_count += len(splited_terrain_components[i])
    rand_num = random.randint(0, components_count - 1)
    component = None
    for i in range(1, 5):
        if rand_num < len(splited_terrain_components[i]):
            component = splited_terrain_components[i][rand_num]
            break
        rand_num -= len(splited_terrain_components[i])
    grid_type_count = [0 for _ in range(5)]
    for x, y in component:
        grid_type_count[grid[x][y]] += 1
    grid_type_count[0] = -1
    max_type = np.argmax(grid_type_count)
    for x, y in component:
        grid[x][y] = max_type

    return grid


def crossover(parent1: List[List[int]], parent2: List[List[int]], n: int) -> Tuple[List[List[int]], List[List[int]]]:
    components1, components2 = find_connected_components(parent1, {}, n), find_connected_components(parent2, {}, n)

    randx, randy, comp_idx1, comp_idx2, type1, type2 = [None for _ in range(6)]
    success = False
    for k in range(10):
        randx, randy = random.randint(0, W - 1), random.randint(0, H - 1)
        comp_idx1, comp_idx2 = None, None
        for i in range(n):
            for j in range(len(components1[i])):
                if (randx, randy) in components1[i][j]:
                    type1 = i
                    comp_idx1 = (i, j)
                    break
            if comp_idx1 != None:
                break
        for i in range(n):
            for j in range(len(components2[i])):
                if (randx, randy) in components2[i][j]:
                    type2 = i
                    comp_idx2 = (i, j)
                    break
            if comp_idx2 != None:
                break
        if type1 != type2:
            success = True
            break
    if not success:
        return parent1, parent2
    comp1, comp2 = components1[comp_idx1[0]][comp_idx1[1]], components2[comp_idx2[0]][comp_idx2[1]]
    intersection = set(comp1).intersection(set(comp2))
    intersection_rate = len(intersection) / (len(comp1) + len(comp2) - len(intersection))

    use_approx = random.random() < CROSSOVER_APPROX_RATE
    if use_approx:
        if intersection_rate < CROSSOVER_APPROX_ACCEPT_RATE:
            return parent1, parent2
        else:
            for x, y in comp1:
                parent1[x][y] = type2
            for x, y in comp2:
                parent2[x][y] = type1
            return parent1, parent2
    else:
        for x, y in intersection:
            parent1[x][y] = type2
            parent2[x][y] = type1
        return parent1, parent2


def mutation(grid: List[List[int]], n: int) -> List[List[int]]:
    randw, randh = random.randint(MUTATION_MINL, MUTATION_MAXL), random.randint(MUTATION_MINL, MUTATION_MAXL)
    randx, randy = random.randint(0, W - randw), random.randint(0, H - randh)
    region_mutation = random.random() < REGION_MUTATION_RATE
    consistent_mutation = random.random() < MUTATION_CONSISTENT_RATE
    if region_mutation:
        components = find_connected_components(grid, {}, n)
        components_count = 0
        for i in range(n):
            components_count += len(components[i])
        rand_idx = random.randint(0, components_count - 1)
        component_type, component = None, None
        for i in range(n):
            if rand_idx < len(components[i]):
                component_type = i
                component = components[i][rand_idx]
                break
            rand_idx -= len(components[i])
        new_type = random.randint(0, n - 1)
        while new_type == component_type:
            new_type = random.randint(0, n - 1)
        for x, y in component:
            grid[x][y] = new_type
    else:
        if consistent_mutation:
            randtype = None
            adjacent_types = set()
            for i in range(randx - 1, randx + randw + 1):
                for j in range(randy - 1, randy + randh + 1):
                    if (
                        0 <= i < W
                        and (i == randx - 1 or i == randx + randw)
                        and 0 <= j < H
                        and (j == randy - 1 or j == randy + randh)
                    ):
                        adjacent_types.add(grid[i][j])
            if len(adjacent_types) == 0:
                randtype = random.randint(0, n - 1)
            else:
                randtype = random.choice(list(adjacent_types))
            for i in range(randw):
                for j in range(randh):
                    grid[randx + i][randy + j] = randtype
        else:
            randtype = random.randint(0, n - 1)
            for i in range(randw):
                for j in range(randh):
                    grid[randx + i][randy + j] = randtype
    return grid


def genetic_algorithm(
    parameters: dict,
    n: int, # 5
    fitness_func: Callable,
    fix_func: Callable,
    fitness_thres: float, # 100
    max_generation: int, # 100
    mutation_rate: float, # 0.7
    crossover_rate: float, # 0.9
    terrain=None,
    infrastructure=None,
    use_fix: bool = True,
) -> Tuple[List[List[int]], float]:
    population = []
    # Initialize 100 population
    for i in range(POPULATION_SIZE): # population size = 100
        population.append(randominit(n, 5))
    max_fitness, max_grid = 0, None

    # Get the max fitness and the corresponding grid
    for i in range(max_generation): # max_generation = 100
        fitness_list = []
        fitness_sum = 0 
        local_max_fitness = 0 # get the best fitness in current generation
        for j in range(POPULATION_SIZE):
            # fitness = threads[j].get_result()
            if fitness_func == terrain_fitness_with_loc:
                fitness = fitness_func(population[j], parameters, terrain, infrastructure, i)
            else:
                fitness = fitness_func(population[j], parameters, terrain, infrastructure)
            fitness_list.append(fitness)
            fitness_sum += fitness
            if fitness > local_max_fitness:
                local_max_fitness = fitness
            if fitness > max_fitness:
                max_fitness = fitness
                max_grid = deepcopy(population[j])
            elif fitness == max_fitness:
                if random.random() < 0.5:
                    max_grid = deepcopy(population[j])
        if max_fitness - fitness_thres > -1e-6: # if get perfect terrain grid
            break
        print("generation", i, "max_fitness", max_fitness)
        new_population = []

        # NOTE deterministic selection
        for j in range(POPULATION_SIZE):
            times = int(fitness_list[j] * POPULATION_SIZE / fitness_sum)
            for k in range(times):
                new_population.append(deepcopy(population[j]))
            # a form of "remainder stochastic sampling" 
            # where the fractional parts of selection probabilities are handled 
            fitness_list[j] -= times * fitness_sum / POPULATION_SIZE #
        fitness_list_with_index = list(enumerate(fitness_list)) # add index for each fitness
        fitness_list_with_index.sort(key=lambda x: x[1], reverse=True) # 
        # Fill remaining slots (except preserved best ones)
        len_now = len(new_population)
        for j in range(POPULATION_SIZE - len_now - PRESERVE_BEST_SIZE):
            new_population.append(deepcopy(population[fitness_list_with_index[j][0]]))
        # Fill any remaining slots with the best solution found so far
        len_now = len(new_population)
        for j in range(POPULATION_SIZE - len_now):
            new_population.append(deepcopy(max_grid))

        # NOTE crossover
        random.shuffle(new_population)
        for j in range(0, POPULATION_SIZE, 2):
            if random.random() < crossover_rate:
                new_population[j], new_population[j + 1] = crossover(new_population[j], new_population[j + 1], n)
        # NOTE mutation
        for j in range(POPULATION_SIZE):
            if random.random() < mutation_rate:
                new_population[j] = mutation(new_population[j], n)
        # NOTE fix
        if use_fix:
            for j in range(POPULATION_SIZE):
                if random.random() < FIX_RATE:
                    new_population[j] = fix_func(new_population[j], parameters, terrain, infrastructure)
        population = new_population
    return max_grid, max_fitness


def generate_terrain(args, parameters: dict) -> List[List[int]]:
    if args.use_conlan:
        return genetic_algorithm(
            parameters, 5, terrain_fitness, terrain_evo, 100, MAX_GENERATION, MUTATION_RATE, CROSSOVER_RATE
        )[0]
    else:
        return genetic_algorithm(
            parameters, 5, terrain_fitness_with_loc, terrain_evo, 100, MAX_GENERATION, MUTATION_RATE, CROSSOVER_RATE
        )[0]


def generate_infrastructure(
    args, parameters: dict, terrain: List[List[int]]
) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]], dict]:
    inf_entrance_num, inf_keyunit_num, inf_road_complexity = (
        deepcopy(parameters["inf_entrance_num"]),
        deepcopy(parameters["inf_keyunit_num"]),
        deepcopy(parameters["inf_road_complexity"]),
    )

    ### generate entrances
    #region
    _, boundary_points = get_boundary(terrain, True)
    boundary_point_adj_types = [] # each point's tblr area terrain type account
    boundary_point_towards = []
    boundary_edges = []
    dx_dys = [(0, 0), (0, -1), (-1, 0), (-1, -1)]
    for x, y in boundary_points:
        if H * x - W * y <= 0 and H * x + W * y <= W * H:  # left
            boundary_point_towards.append(0)
        elif H * x - W * y <= 0 and H * x + W * y > W * H:  # top
            boundary_point_towards.append(1)
        elif H * x - W * y > 0 and H * x + W * y > W * H:  # right
            boundary_point_towards.append(2)
        else:  # bottom
            boundary_point_towards.append(3)
        adjacent_type_cnt = [0 for _ in range(5)]
        for dx, dy in dx_dys:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < W and 0 <= new_y < H:
                adjacent_type_cnt[terrain[new_x][new_y]] += 1
            else:
                adjacent_type_cnt[0] += 1
        boundary_point_adj_types.append(adjacent_type_cnt)
    for i in range(len(boundary_points)):
        boundary_edges.append([boundary_points[i - 1], boundary_points[i]])

    if inf_entrance_num == -1:
        inf_entrance_num = (-1, -1)
    lo, hi = inf_entrance_num
    if hi == -1:
        hi = 6 if lo == -1 else max(lo, 6)
    if lo == -1:
        lo = 3 if hi == -1 else min(hi, 3)
    hi, lo = max(min(hi, 8), 3), max(min(lo, 8), 3)
    entrance_num = random.randint(lo, hi)

    min_loss, min_entrance_point_idxs = 1e9, []
    for k in range(1000):
        loss = 0
        entrance_point_idxs = random.sample(range(len(boundary_points)), entrance_num)
        entrance_point_idxs.sort()
        idx_diff_thres = len(boundary_points) / (entrance_num * 1.5)
        for i in range(1, len(entrance_point_idxs)):
            if entrance_point_idxs[i] - entrance_point_idxs[i - 1] < idx_diff_thres:  # too close
                loss += idx_diff_thres - entrance_point_idxs[i] + entrance_point_idxs[i - 1]
        if entrance_point_idxs[0] + len(boundary_points) - entrance_point_idxs[-1] < idx_diff_thres:  # too close
            loss += idx_diff_thres - entrance_point_idxs[0] + len(boundary_points) - entrance_point_idxs[-1]

        towards_count = [0 for _ in range(4)]
        for idx in entrance_point_idxs:
            if boundary_point_adj_types[idx][0] >= 3:
                loss += 20
            others_cnt = sum(boundary_point_adj_types[idx][1:])
            if (
                boundary_point_adj_types[idx][1] == 1 or boundary_point_adj_types[idx][4] == others_cnt
            ):  # cannot be adjacent to pure lake or hill
                loss += 10

            towards_count[boundary_point_towards[idx]] += 1
        if max(towards_count) - min(towards_count) > 1:  # towards should be balanced
            loss += (max(towards_count) - min(towards_count)) * 10

        if loss < min_loss:
            min_loss = loss
            min_entrance_point_idxs = entrance_point_idxs
    entrance_points = [boundary_points[idx] for idx in min_entrance_point_idxs]
    #endregion

    ### generate keyunits
    #region
    if inf_keyunit_num == -1:
        inf_keyunit_num = (-1, -1)
    lo, hi = inf_keyunit_num
    if hi == -1:
        hi = 4 if lo == -1 else max(lo, 4)
    if lo == -1:
        lo = 2 if hi == -1 else min(hi, 2)
    hi, lo = max(min(hi, 6), 2), max(min(lo, 6), 2)
    keyunits_num = random.randint(lo, hi)

    min_loss, min_keyunit_idxs = 1e9, []
    available_points = []
    for i in range(W):
        for j in range(H):
            if terrain[i][j] > 1:
                available_points.append((i, j))
    for k in range(1000):
        loss = 0
        keyunit_idxs = random.sample(range(len(available_points)), keyunits_num)
        keyunits = [available_points[idx] for idx in keyunit_idxs]
        keyunit_types_count = [0 for _ in range(5)]
        for i in range(len(keyunits)):
            keyunit_types_count[terrain[keyunits[i][0]][keyunits[i][1]]] += 1

            min_boundary_dis = 1e9
            for boundary_point in boundary_points:
                min_boundary_dis = min(min_boundary_dis, unit_corner_dis(keyunits[i], boundary_point))
            if min_boundary_dis <= W // 10:  # should not be too close to boundary
                loss += (W // 10 + 1 - min_boundary_dis) * 10

            min_entrance_dis = 1e9
            for entrance in entrance_points:
                min_entrance_dis = min(min_entrance_dis, unit_corner_dis(keyunits[i], entrance))
            if min_entrance_dis <= W // 10:  # should not be too close to entrance
                loss += (W // 10 + 1 - min_entrance_dis) * 10
            if min_entrance_dis > W // 4:  # should not be too far from entrance
                loss += (min_entrance_dis - W // 4) * 3

            min_another_keyunit_dis, max_another_keyunit_dis = 1e9, 0
            for j in range(len(keyunits)):
                if i == j:
                    continue
                another_keyunit_dis = abs(keyunits[i][0] - keyunits[j][0]) + abs(keyunits[i][1] - keyunits[j][1])
                min_another_keyunit_dis = min(min_another_keyunit_dis, another_keyunit_dis)
                max_another_keyunit_dis = max(max_another_keyunit_dis, another_keyunit_dis)
            if max_another_keyunit_dis >= min_another_keyunit_dis * 2:  # distance should be balanced
                loss += (max_another_keyunit_dis - min_another_keyunit_dis * 2) * 2

        keyunit_types_count[0] = max(keyunit_types_count)  # special case for unused
        if max(keyunit_types_count) - min(keyunit_types_count) > 1:  # types should be balanced
            loss += (max(keyunit_types_count) - min(keyunit_types_count)) * 20
        if loss < min_loss:
            min_loss = loss
            min_keyunit_idxs = keyunit_idxs
    keyunits = [available_points[idx] for idx in min_keyunit_idxs] # [(x1,y1)...]
    #endregion

    ### generate mainroads
    #region
    # evaluate every edge and corner
    # the unused edge is not assigned in edge_scores
    edge_scores = {} # W+1+H+1 tuples - unsued edge, inilization; -10: whole scene boundary, 0:inner area, 2:area boundary; it will increase
    for i in range(W):
        for j in range(H + 1):
            # NOTE add boundary edge
            if j == 0:
                if terrain[i][j] > 0:
                    edge_scores[((i, j), (i + 1, j))] = -10 # Bottom edge, and the terrain is not unused one
            elif j == H:
                if terrain[i][j - 1] > 0:
                    edge_scores[((i, j), (i + 1, j))] = -10 # Top edge
            # NOTE add the edge near border for corridor
            elif j == 1 and i > 0 and i < W-1:
                if terrain[i][j] > 0 and terrain[i][0] > 0:     
                    edge_scores[((i, j), (i + 1, j))] = 1   # secoundary bottom edge
                elif terrain[i][j] > 0:
                    edge_scores[((i, j), (i + 1, j))] = -10 # bottom edge
            elif j == H-1 and i > 0 and i < W-1:
                if terrain[i][j - 1] > 0 and terrain[i][j] > 0: 
                    edge_scores[((i, j), (i + 1, j))] = 1   # secoundary top edge 
                elif terrain[i][j - 1] > 0:
                    edge_scores[((i, j), (i + 1, j))] = -10 # top edge
            # NOTE add inner edge
            else:
                if terrain[i][j - 1] != terrain[i][j]:          # up and down terrain is different
                    # HACK simply remove the lakeside corridor
                    if terrain[i][j] > 0 and terrain[i][j - 1] > 0: 
                        edge_scores[((i, j), (i + 1, j))] = 2   # up and down terrain is both used
                    else:
                        edge_scores[((i, j), (i + 1, j))] = -10 # up and down terrain contains unused one
                else:
                    if terrain[i][j] > 0:
                        edge_scores[((i, j), (i + 1, j))] = 0   # up and down terrain is same, edge in area
    for i in range(W + 1):
        for j in range(H):
            # NOTE
            if i == 0:
                if terrain[i][j] > 0:
                    edge_scores[((i, j), (i, j + 1))] = -10
            elif i == W:
                if terrain[i - 1][j] > 0:
                    edge_scores[((i, j), (i, j + 1))] = -10
            elif i == 1 and j > 0 and j < H-1:
                if terrain[i][j] > 0 and terrain[0][j] > 0:
                    edge_scores[((i, j), (i, j + 1))] = 1   # secoundary left 
                elif terrain[i][j] > 0:
                    edge_scores[((i, j), (i + 1, j))] = -10 # left
            elif i == W-1 and  j > 0 and j < H-1:
                if terrain[i - 1][j] > 0 and terrain[i][j] > 0:
                    edge_scores[((i, j), (i, j + 1))] = 1   # secoundary right
                elif terrain[i - 1][j] > 0:
                    edge_scores[((i, j), (i, j + 1))] = -10 # right
            else:
                if terrain[i - 1][j] != terrain[i][j]:
                    # HACK same
                    if terrain[i][j] > 0 and terrain[i - 1][j] > 0:
                        edge_scores[((i, j), (i, j + 1))] = 2
                    else:
                        edge_scores[((i, j), (i, j + 1))] = -10
                else:
                    if terrain[i][j] > 0:
                        edge_scores[((i, j), (i, j + 1))] = 0
    for keyunit in keyunits:
        edges = [
            ((keyunit[0], keyunit[1]), (keyunit[0] + 1, keyunit[1])),           # bottom
            ((keyunit[0], keyunit[1]), (keyunit[0], keyunit[1] + 1)),           # left
            ((keyunit[0] + 1, keyunit[1]), (keyunit[0] + 1, keyunit[1] + 1)),   # right
            ((keyunit[0], keyunit[1] + 1), (keyunit[0] + 1, keyunit[1] + 1)),   # top
        ]
        for edge in edges:
            if edge in edge_scores:
                edge_scores[edge] += 1  # add the score of edge around the keyunits
    corner_scores = {} # edge
    for i in range(W + 1):
        for j in range(H + 1):
            edges = [
                ((i, j), (i + 1, j)),
                ((i, j), (i, j + 1)),
                ((i - 1, j), (i, j)),
                ((i, j - 1), (i, j)),
            ]
            score = 0
            for k in range(4):
                if edges[k] not in edge_scores: # is not the corner
                    score = -10
                else:
                    score += edge_scores[edges[k]]
                if (edges[k] in edge_scores) and (edges[k - 1] in edge_scores):
                    if edge_scores[edges[k]] >= 2 and edge_scores[edges[k - 1]] >= 2:
                        score += 3
                    elif edge_scores[edges[k]] < 0 and edge_scores[edges[k - 1]] < 0:
                        score += 2
            corner_scores[(i, j)] = score

    # use keypoints to represent keyunits
    keypoints = []
    for keyunit in keyunits:
        candidates = [
            (keyunit[0], keyunit[1]),
            (keyunit[0] + 1, keyunit[1]),
            (keyunit[0], keyunit[1] + 1),
            (keyunit[0] + 1, keyunit[1] + 1),
        ]
        random.shuffle(candidates)
        candidates.sort(key=lambda x: corner_scores[x], reverse=True)
        keypoints.append(candidates[0])

    # use union-find
    uf = UnionFind([(i, j) for i in range(W + 1) for j in range(H + 1)])
    edge_used = {}  # 0: not used, 1: mainroad, 2: secondary road
    for key in edge_scores:
        edge_used[key] = 0
    # connect every entrance to the nearest keyunit
    for i in range(entrance_num):
        min_dis, min_keyunit_idx = 1e9, -1
        for j in range(keyunits_num):
            dis = abs(entrance_points[i][0] - keypoints[j][0]) + abs(entrance_points[i][1] - keypoints[j][1])
            if dis < min_dis:
                min_dis = dis
                min_keyunit_idx = j
        uf, edge_used = connect_corners(entrance_points[i], keypoints[min_keyunit_idx], uf, edge_used, edge_scores) # set the main road
    # connect keyunits, from the nearest to the farthest
    keypoint_dists = []
    for i in range(keyunits_num):
        for j in range(i + 1, keyunits_num):
            keypoint_dists.append(
                (abs(keypoints[i][0] - keypoints[j][0]) + abs(keypoints[i][1] - keypoints[j][1]), i, j)
            )
    keypoint_dists.sort(key=lambda x: x[0])
    for dist, i, j in keypoint_dists:
        uf, edge_used = connect_corners(keypoints[i], keypoints[j], uf, edge_used, edge_scores)
    # handle isolated keyunits (with degree 1)
    for i in range(keyunits_num):
        px, py = keypoints[i]
        degree = 0
        edges = [
            ((px - 1, py), (px, py)),
            ((px, py), (px, py + 1)),
            ((px, py), (px + 1, py)),
            ((px, py - 1), (px, py)),
        ]
        for edge in edges:
            if edge in edge_used and edge_used[edge] == 1:
                degree += 1
        if degree == 1:
            dist_to_boundary_points = [
                (abs(px - boundary_point[0]) + abs(py - boundary_point[1]), boundary_point)
                for boundary_point in boundary_points
            ]
            dist_to_boundary_points.sort(key=lambda x: x[0])
            for dist, boundary_point in dist_to_boundary_points:
                if (boundary_point in entrance_points) or (boundary_point in keypoints):
                    continue
                uf, edge_used = connect_corners(keypoints[i], boundary_point, uf, edge_used, edge_scores)
                break
    #endregion

    ### generate secondary roads
    #region
    if inf_road_complexity == -1:
        inf_road_complexity = (-1, -1)
    lo, hi = inf_road_complexity
    if hi == -1:
        hi = 0.4 if lo == -1 else max(lo, 0.4)
    if lo == -1:
        lo = 0.35 if hi == -1 else min(hi, 0.35)
    hi, lo = max(min(hi, 0.4), 0.35), max(min(lo, 0.4), 0.35)
    #hi, lo = 0.05, 0.03
    road_complexity = random.uniform(lo, hi) # decrease the complexity
    print(road_complexity)

    # add all remaining borders
    for edge in edge_scores:
        if (edge_scores[edge] >= 2 or edge_scores[edge] < 0) and edge_used[edge] == 0:
            edge_used[edge] = 2

    # split regions and add secondary roads until complexity is satisfied
    main_roads, secondary_roads = [], []
    for edge in edge_used:
        if edge_used[edge] == 1:
            main_roads.append(edge)
        elif edge_used[edge] == 2:
            secondary_roads.append(edge)
    while (len(main_roads) + len(secondary_roads)) / len(edge_used) < road_complexity:
        components = find_connected_components(terrain, edge_used, 5)
        max_component_size, max_component_idx = 0, None
        for i in range(len(components)):
            if i == 0:  # unused
                continue
            for j in range(len(components[i])):
                if len(components[i][j]) > max_component_size:
                    max_component_size = len(components[i][j])
                    max_component_idx = (i, j)
        component = components[max_component_idx[0]][max_component_idx[1]]
        split_edges = growth_split(component)
        if len(split_edges) == 0:
            break
        for edge in split_edges:
            edge_used[edge] = 2
            secondary_roads.append(edge)
    #endregion

    return (entrance_points, keyunits, edge_used)



def generate_attributes(
    args,
    parameters: dict,
    terrain: List[List[int]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[Tuple[int, int], Tuple[int, int]]], dict],
) -> List[List[int]]:
    if args.use_conlan:
        return genetic_algorithm(
            parameters,
            5,
            attributes_fitness,
            attributes_evo,
            100,
            MAX_GENERATION,
            MUTATION_RATE,
            CROSSOVER_RATE,
            terrain=terrain,
            infrastructure=infrastructure,
        )[0]
    else: 
        return genetic_algorithm(
            parameters,
            5,
            attributes_fitness,
            attributes_evo,
            5,
            MAX_GENERATION,
            MUTATION_RATE,
            CROSSOVER_RATE,
            terrain=terrain,        # Give the terrain and infrastructure
            infrastructure=infrastructure,
        )[0]


def generate_heightmap(
    parameters: dict,
    terrain: List[List[int]],
) -> List[List[int]]:
    # generate height map
    labels = [[0 for _ in range(H)] for _ in range(W)]
    vals = [[0 for _ in range(H)] for _ in range(W)]
    ## add noise for 0-3 terrain
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 0:  # unused
                vals[i][j] = WATER_HEIGHT
            elif terrain[i][j] == 1:  # lake
                vals[i][j] = 0
            elif terrain[i][j] == 2:  # land
                vals[i][j] = 0.1 + random.random() * 0.025
            elif terrain[i][j] == 3:  # ground
                vals[i][j] = 0.1 + random.random() * 0.01
    near_4 = [[0, 1], [0, -1], [1, 0], [-1, 0]]
    ## add noise for hill
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 4:  # hill
                isfirst = False
                for dx, dy in near_4:
                    new_x, new_y = i + dx, j + dy
                    if 0 <= new_x < W and 0 <= new_y < H and terrain[new_x][new_y] != 4:
                        isfirst = True
                        break
                if isfirst:
                    labels[i][j] = 1
                    vals[i][j] = 0.15 + random.random() * 0.3
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 4 and labels[i][j] == 0:
                issecond = False
                for dx, dy in near_4:
                    new_x, new_y = i + dx, j + dy
                    if 0 <= new_x < W and 0 <= new_y < H and terrain[new_x][new_y] == 4 and labels[new_x][new_y] == 1:
                        issecond = True
                        break
                if issecond:
                    labels[i][j] = 2
                    vals[i][j] = 0.4 + random.random() * 0.3
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 4 and labels[i][j] == 0:
                labels[i][j] = 3
                vals[i][j] = 0.6 + random.random() * 0.4
    near_4_corner = [[0, 0], [0, -1], [-1, 0], [-1, -1]]
    expanded_vals = [[WATER_HEIGHT for _ in range(H + 2)] for _ in range(W + 2)] # considering the buffer area
    for i in range(W):
        for j in range(H):
            expanded_vals[i + 1][j + 1] = vals[i][j]

    corner_vals = [[-1 for _ in range(2 * H + 5)] for _ in range(2 * W + 5)] # (0,2*W+4) = (0,2*(W+2))
    ## corner_val bilinear interpolation (unit to corner, 4 points to 1 point)
    for i in range(W + 3):
        for j in range(H + 3):
            total_val, total_count = 0, 0
            bay_cnt, lake_cnt = 0, 0
            for dx, dy in near_4_corner:
                new_x, new_y = i + dx, j + dy
                if 0 <= new_x < W + 2 and 0 <= new_y < H + 2:
                    total_val += expanded_vals[new_x][new_y]
                    if expanded_vals[new_x][new_y] >= 0.1 - 1e-4:
                        bay_cnt += 1
                    if expanded_vals[new_x][new_y] == 0: # in the lake
                        lake_cnt += 1
                    total_count += 1
                else:
                    total_count += 1
            if bay_cnt and lake_cnt: # the junction between water and land, ground or hill
                corner_vals[2 * i][2 * j] = total_val / (total_count - lake_cnt)
            else:
                corner_vals[2 * i][2 * j] = total_val / total_count

    ## special handle for water
    ### if not, probably 
    for i in range(W + 2):
        for j in range(H + 2):
            if expanded_vals[i][j] == 0:
                corner_vals[2 * i + 1][2 * j + 1] = 0 # center point
    for i in range(W):
        for j in range(1, H):
            u1, u2 = terrain[i][j - 1], terrain[i][j]
            if u1 == 1 and u2 == 1:  # connected lake
                corner_vals[2 * i + 3][2 * j + 2] = 0
    for i in range(1, W):
        for j in range(H):
            u1, u2 = terrain[i - 1][j], terrain[i][j]
            if u1 == 1 and u2 == 1:  # connected lake
                corner_vals[2 * i + 2][2 * j + 3] = 0

    ## bilinear interpolation (corner to other corner), complete other corners
    for i in range(W + 3):
        for j in range(H + 3):
            if i < W + 2:
                if corner_vals[2 * i + 1][2 * j] == -1:
                    corner_vals[2 * i + 1][2 * j] = (corner_vals[2 * i][2 * j] + corner_vals[2 * i + 2][2 * j]) / 2
            if j < H + 2:
                if corner_vals[2 * i][2 * j + 1] == -1:
                    corner_vals[2 * i][2 * j + 1] = (corner_vals[2 * i][2 * j] + corner_vals[2 * i][2 * j + 2]) / 2
            if i < W + 2 and j < H + 2:
                if corner_vals[2 * i + 1][2 * j + 1] == -1:
                    corner_vals[2 * i + 1][2 * j + 1] = (
                        corner_vals[2 * i][2 * j]
                        + corner_vals[2 * i + 2][2 * j]
                        + corner_vals[2 * i][2 * j + 2]
                        + corner_vals[2 * i + 2][2 * j + 2]
                    ) / 4

    ## add a perlin noise to smooth
    ## MAP_W*MAP_H units
    height_map = [[0 for _ in range(MAP_H)] for _ in range(MAP_W)]
    perlin_noise = [[0 for _ in range(MAP_H)] for _ in range(MAP_W)]
    try:
        perlin_noise = generate_fractal_noise_2d((MAP_W, MAP_H), (4, 4))
    except:
        pass
    # bilinear_interpolation(height_map, corner_vals, perlin_noise)
    bicubic_interpolation(height_map, corner_vals, perlin_noise)
    return height_map

def decide_edge_info(
    ed: Tuple[Tuple[int, int], Tuple[int, int]],
    u1: int,
    u2: int,
    edge_used: dict,
    entrance_points: List[Tuple[int, int]],
    order: str
) -> List[int]:
    """
    extend the 3 types for edge to 10 types
    """
    info_list = []
    if u1 == 0 or u2 == 0:
        if ed[0] in entrance_points or ed[1] in entrance_points:
            info_list.append(6)
        else:
            info_list.append(0)
        # if 0 < u1 < 4 or 0 < u2 < 4:
        #     info_list.append(3)
    else:
        v1, v2 = min(u1, u2), max(u1, u2)
        if edge_used[ed] == 0:      # not a road
            if v1 != v2:
                info_list.append(3)     ## boundary
        elif edge_used[ed] == 1:    # main road
            info_list.append(1)
            if v1 < 4 and v2 < 4:       ## bridge
                info_list.append(5)
            elif v1 < 4 and v2 >= 4:    ## lakeside
                info_list.append(4)
                # add waterside direction
                if order == "width":
                    if u1 < 4 and u2 >= 4:
                        info_list.append(9) ### water is down
                    else:
                        info_list.append(7) ### water is up
                elif order == "height":
                    if u1 < 4 and u2 >= 4:
                        info_list.append(8) ### water is left
                    else:
                        info_list.append(10) ### water is right
        elif edge_used[ed] == 2:    # secondary road
            info_list.append(2)
            if v1 < 4 and v2 < 4:       ## bridge
                info_list.append(5)
            elif v1 < 4 and v2 >= 4:    ## lakeside
                info_list.append(4)
                # add waterside direction
                if order == "width":
                    if u1 < 4 and u2 >= 4:
                        info_list.append(9) ### water is down
                    else:
                        info_list.append(7) ### water is up
                elif order == "height":
                    if u1 < 4 and u2 >= 4:
                        info_list.append(8) ### water is left
                    else:
                        info_list.append(10) ### water is right
        
    info_list.sort()
    # if info_list is empty, the edge is not boundary, road.
    return info_list


def bilinear_interpolation(height_map: List[List[float]], corner_vals: List[List[float]]) -> None:
    # MAX_W, MAX_H, to W, H + 5
    for i in tqdm(range(MAP_W)):
        for j in range(MAP_H):
            x, y = (i + 0.5) * (2 * W + 4) / MAP_W, (j + 0.5) * (2 * W + 4) / MAP_H
            x1, y1 = int(x), int(y)
            x2, y2 = x1 + 1, y1 + 1
            x, y = x - x1, y - y1
            w1, w2, w3, w4 = (1 - x) * (1 - y), (1 - x) * y, x * (1 - y), x * y
            height_map[i][j] = max(
                min(
                    (
                        w1 * corner_vals[x1][y1]
                        + w2 * corner_vals[x1][y2]
                        + w3 * corner_vals[x2][y1]
                        + w4 * corner_vals[x2][y2]
                    ),
                    1,
                ),
                0,
            )

def bicubic_interpolation(
    height_map: List[List[float]],
    corner_vals: List[List[float]],
    noise: List[List[float]],
) -> None:
    # This creates a spline object that can be evaluated at any point within the range of original grid.
    interpolation = interp.RectBivariateSpline(range(2 * W + 5), range(2 * H + 5), corner_vals, kx=3, ky=3)
    # MAP_H to H + 5
    for i in tqdm(range(MAP_W)):
        for j in range(MAP_H):
            x, y = (i + 0.5) * (2 * W + 4) / MAP_W, (j + 0.5) * (2 * H + 4) / MAP_H
            height_map[i][j] = max(
                min(
                    interpolation(x, y)[0][0], # + (noise[i][j] - 0.5) * 0.015,
                    1,
                ),
                0,
            )


def make_continuous(
    terrain: List[List[int]],
    attributes: List[List[int]],
    infrastructure: Tuple[List[Tuple[int, int]], List[Tuple[Tuple[int, int], Tuple[int, int]]], dict],
    gen_idx: int,
) -> None:
    """
    combined: terrain+attributes
    0: unused, 1: water, 2: lotus, 3: lake rock, 4: pure land, 5: grass/flower, 6: shrub, 7: tree,
    8: pavilion, 9: plaza, 10: flowerbed, 11: tree lines, 12: building, 13: hill rock, 14: grass
    edge info: 0: wall, 1: main road, 2: secondary road, 3: type border, 4: lakeside, 5: bridge, 6: wall with hole; 
    edge info for waterside direction relation: 7: 0pi z-forward, water is up; 8: -x-forward, left; 9: -z-forward, down; 10: x-forward, ; 
    Note: the object's default direction is z-forward
    """
    entrance_points, keyunits, edge_used = infrastructure

    # get combined attribute types
    combined = [[0 for _ in range(H)] for _ in range(W)]
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 0:  # unused
                combined[i][j] = 0
            elif terrain[i][j] == 1:  # lake
                if attributes[i][j] == 0:  # water
                    combined[i][j] = 1
                elif attributes[i][j] == 1:  # lotus
                    combined[i][j] = 2
                elif attributes[i][j] == 2:  # lake rock
                    combined[i][j] = 3
                elif attributes[i][j] == 3:  # not valid, fixed to water
                    combined[i][j] = 1
                elif attributes[i][j] == 4:  # lake rock
                    combined[i][j] = 3
            elif terrain[i][j] == 2:  # land
                if attributes[i][j] == 0:  # pure land
                    combined[i][j] = 4
                elif attributes[i][j] == 1:  # grass/flower
                    combined[i][j] = 5
                elif attributes[i][j] == 2:  # shrub
                    combined[i][j] = 6
                elif attributes[i][j] == 3:  # tree
                    combined[i][j] = 7
                elif attributes[i][j] == 4:  # pavilion
                    combined[i][j] = 8
            elif terrain[i][j] == 3:  # ground
                if attributes[i][j] == 0:  # plaza
                    combined[i][j] = 9
                elif attributes[i][j] == 1:  # plaza
                    combined[i][j] = 9
                elif attributes[i][j] == 2:  # shrub spots
                    combined[i][j] = 10
                elif attributes[i][j] == 3:  # tree lines
                    combined[i][j] = 11
                elif attributes[i][j] == 4:  # building
                    combined[i][j] = 12
            elif terrain[i][j] == 4:  # hill
                if attributes[i][j] == 0:  # hill rock
                    combined[i][j] = 13
                elif attributes[i][j] == 1:  # grass
                    combined[i][j] = 14
                elif attributes[i][j] == 2:  # shrub
                    combined[i][j] = 6
                elif attributes[i][j] == 3:  # tree
                    combined[i][j] = 7
                elif attributes[i][j] == 4:  # pavilion
                    combined[i][j] = 8

    # get edge info
    combined_components = find_connected_components(combined, edge_used, 15) # 15 should be the length of combined types
    edge_info = {} # ((startX, startY),(endX, endY): [0-6],...)
    ## the edge parallel to width and height boundary line
    for i in range(W):
        for j in range(H + 1):
            ed = ((i, j), (i + 1, j))
            if ed not in edge_used:
                continue
            info_list = [] # contains 0-6, presenting the object in the edge, such as bridge or entrance point
            if j == 0: # for first W edge
                if combined[i][j] > 0:
                    if ed[0] in entrance_points or ed[1] in entrance_points:
                        info_list.append(6)
                    else:
                        info_list.append(0)
                # if 0 < combined[i][j] < 4:
                #     info_list.append(3)
            elif j == H: # for end W edge
                if combined[i][j - 1] > 0:
                    if ed[0] in entrance_points or ed[1] in entrance_points:
                        info_list.append(6)
                    else:
                        info_list.append(0)
                # if 0 < combined[i][j - 1] < 4:
                #     info_list.append(3)
            else:
                info_list = decide_edge_info(ed, combined[i][j - 1], combined[i][j], edge_used, entrance_points, "width")
            if len(info_list) > 0:
                edge_info[ed] = info_list
    for i in range(W + 1):
        for j in range(H):
            ed = ((i, j), (i, j + 1))
            if ed not in edge_used:
                continue
            info_list = []
            if i == 0:
                if combined[i][j] > 0:
                    if ed[0] in entrance_points or ed[1] in entrance_points:
                        info_list.append(6)
                    else:
                        info_list.append(0)
            elif i == W:
                if combined[i - 1][j] > 0:
                    if ed[0] in entrance_points or ed[1] in entrance_points:
                        info_list.append(6)
                    else:
                        info_list.append(0)
            else:
                info_list = decide_edge_info(ed, combined[i - 1][j], combined[i][j], edge_used, entrance_points, "height")
            if len(info_list) > 0:
                edge_info[ed] = info_list

    ################# generate height map ################
    labels = [[0 for _ in range(H)] for _ in range(W)]
    vals = [[0 for _ in range(H)] for _ in range(W)]
    ## add noise for 0-3 terrain
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 0:  # unused
                vals[i][j] = WATER_HEIGHT
            elif terrain[i][j] == 1:  # lake
                vals[i][j] = 0
            elif terrain[i][j] == 2:  # land
                #vals[i][j] = 0.1 + random.random() * 0.025
                vals[i][j] = 0.1
            elif terrain[i][j] == 3:  # ground
                #vals[i][j] = 0.1 + random.random() * 0.01
                vals[i][j] = 0.1
    near_4 = [[0, 1], [0, -1], [1, 0], [-1, 0]]
    ## add noise for hill
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 4:  # hill
                isfirst = False
                for dx, dy in near_4:
                    new_x, new_y = i + dx, j + dy
                    if 0 <= new_x < W and 0 <= new_y < H and terrain[new_x][new_y] != 4:
                        isfirst = True
                        break
                if isfirst:
                    labels[i][j] = 1
                    vals[i][j] = 0.15 + random.random() * 0.3
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 4 and labels[i][j] == 0:
                issecond = False
                for dx, dy in near_4:
                    new_x, new_y = i + dx, j + dy
                    if 0 <= new_x < W and 0 <= new_y < H and terrain[new_x][new_y] == 4 and labels[new_x][new_y] == 1:
                        issecond = True
                        break
                if issecond:
                    labels[i][j] = 2
                    vals[i][j] = 0.4 + random.random() * 0.3
    for i in range(W):
        for j in range(H):
            if terrain[i][j] == 4 and labels[i][j] == 0:
                labels[i][j] = 3
                vals[i][j] = 0.6 + random.random() * 0.4
    near_4_corner = [[0, 0], [0, -1], [-1, 0], [-1, -1]]
    expanded_vals = [[WATER_HEIGHT for _ in range(H + 2)] for _ in range(W + 2)] # considering the buffer area
    for i in range(W):
        for j in range(H):
            expanded_vals[i + 1][j + 1] = vals[i][j] # NOTE, the expanded_vals has been removed to center of the grid

    corner_vals = [[-1 for _ in range(2 * H + 5)] for _ in range(2 * W + 5)] # (0,2*W+4) = (0,2*(W+2))
    ## corner_val bilinear interpolation (unit to corner, 4 points to 1 point)
    for i in range(W + 3):
        for j in range(H + 3):
            total_val, total_count = 0, 0
            bay_cnt, lake_cnt = 0, 0
            for dx, dy in near_4_corner:
                new_x, new_y = i + dx, j + dy
                if 0 <= new_x < W + 2 and 0 <= new_y < H + 2:
                    total_val += expanded_vals[new_x][new_y]
                    if expanded_vals[new_x][new_y] >= 0.1 - 1e-4:
                        bay_cnt += 1
                        #total_val += expanded_vals[new_x][new_y]
                    if expanded_vals[new_x][new_y] == 0: # in the lake
                        lake_cnt += 1
                    total_count += 1
                else:
                    total_count += 1
            if bay_cnt and lake_cnt: # the junction between water and land, ground or hill
                corner_vals[2 * i][2 * j] = total_val / (total_count - lake_cnt)
                #corner_vals[2 * i][2 * j] = total_val / total_count
            else:
                corner_vals[2 * i][2 * j] = total_val / total_count

    ## special handle for water
    ### if not, probably 
    for i in range(W + 2):
        for j in range(H + 2):
            if expanded_vals[i][j] == 0:
                corner_vals[2 * i + 1][2 * j + 1] = 0 # center point
    for i in range(W):
        for j in range(1, H):
            u1, u2 = terrain[i][j - 1], terrain[i][j]
            if u1 == 1 and u2 == 1:  # connected lake
                corner_vals[2 * i + 3][2 * j + 2] = 0 # expanded_val get 1 offset along xy index, corner_vals[2 * (i + 1) + 1][2 * (j + 1)] 

    for i in range(1, W):
        for j in range(H):
            u1, u2 = terrain[i - 1][j], terrain[i][j]
            if u1 == 1 and u2 == 1:  # connected lake
                corner_vals[2 * i + 2][2 * j + 3] = 0

    ## bilinear interpolation (corner to other corner), complete other corners
    for i in range(W + 3):
        for j in range(H + 3):
            if i < W + 2:
                if corner_vals[2 * i + 1][2 * j] == -1:
                    corner_vals[2 * i + 1][2 * j] = (corner_vals[2 * i][2 * j] + corner_vals[2 * i + 2][2 * j]) / 2
            if j < H + 2:
                if corner_vals[2 * i][2 * j + 1] == -1:
                    corner_vals[2 * i][2 * j + 1] = (corner_vals[2 * i][2 * j] + corner_vals[2 * i][2 * j + 2]) / 2
            if i < W + 2 and j < H + 2:
                if corner_vals[2 * i + 1][2 * j + 1] == -1:
                    corner_vals[2 * i + 1][2 * j + 1] = (
                        corner_vals[2 * i][2 * j]
                        + corner_vals[2 * i + 2][2 * j]
                        + corner_vals[2 * i][2 * j + 2]
                        + corner_vals[2 * i + 2][2 * j + 2]
                    ) / 4

    ## add a perlin noise to smooth
    ## MAP_W*MAP_H units
    # DEBUG MAP_H to W + 5, H + 5
    height_map = [[0 for _ in range(MAP_H)] for _ in range(MAP_W)]
    perlin_noise = [[0 for _ in range(MAP_H)] for _ in range(MAP_W)]
    try:
        perlin_noise = generate_fractal_noise_2d((MAP_W, MAP_H), (4, 4))
    except:
        pass
    # bilinear_interpolation(height_map, corner_vals, perlin_noise)
    bicubic_interpolation(height_map, corner_vals, perlin_noise)
    #height_map = corner_vals
    #height_map = [[1 for _ in range(MAP_H)] for _ in range(MAP_W)]


    ############################## generate edge ##################################
    # categorize edges
    edge_group, group_cnt = {}, 0
    group_points_list, group_info_list = [], [] # points in different grouped edge
    area_centers_and_types = []
    for edge in edge_info:
        edge_group[edge] = -1
    ## find the corner have different edges, ending edge
    ## using cross-shaped structure
    intersection_corners = []
    for i in range(W + 1):
        for j in range(H + 1):
            edges = [((i, j), (i + 1, j)), ((i, j), (i, j + 1)), ((i - 1, j), (i, j)), ((i, j - 1), (i, j))]
            edge_cnt, last_info, different = 0, None, False
            for edge in edges:
                if edge in edge_info:
                    edge_cnt += 1
                    if last_info == None:
                        last_info = edge_info[edge]
                    elif last_info != edge_info[edge]: # and len([i for i in water_direction if i in last_info]) == 0 and len([i for i in water_direction if i in edge_info[edge]]) == 0:
                        different = True
            if edge_cnt > 2 or edge_cnt == 1 or different:
                intersection_corners.append((i, j))
    ## Grouping Connected Edges
    for x, y in intersection_corners:
        start_corner = (x, y)
        edges = [((x, y), (x + 1, y)), ((x, y), (x, y + 1)), ((x - 1, y), (x, y)), ((x, y - 1), (x, y))]
        for edge in edges:
            # find the unprocessed edge
            if not (edge in edge_group and edge_group[edge] == -1):
                continue
            # starting from this edge, find all edges in the same group, until meeting the intersection point
            edge_group[edge] = group_cnt # assign the edge index
            now_corner = edge[0] if edge[0] != start_corner else edge[1]
            group_points = [start_corner, now_corner] # from center to Up Down Left Right
            while now_corner not in intersection_corners:
                nx, ny = now_corner
                next_edges = [
                    ((nx, ny), (nx + 1, ny)),
                    ((nx, ny), (nx, ny + 1)),
                    ((nx - 1, ny), (nx, ny)),
                    ((nx, ny - 1), (nx, ny)),
                ]
                for next_edge in next_edges:
                    if (next_edge in edge_group) and edge_group[next_edge] == -1:
                        edge_group[next_edge] = group_cnt
                        now_corner = next_edge[0] if next_edge[0] != now_corner else next_edge[1]
                        group_points.append(now_corner)
                        break
            group_cnt += 1
            group_points_list.append(group_points)
            group_info_list.append(edge_info[edge])

    # HACK get combined type of different areas, through the center point
    for i in range(len(combined_components)):
        for j in range(len(combined_components[i])):
            component = combined_components[i][j]
            minx, miny, maxx, maxy = 1e9, 1e9, -1, -1
            for p in component: # (x,y)
                minx = min(minx, p[0])
                miny = min(miny, p[1])
                maxx = max(maxx, p[0] + 1)
                maxy = max(maxy, p[1] + 1)
            area_centers_and_types.append(((minx + maxx) / 2, (miny + maxy) / 2, i))

    # generate continuous edges, and 
    refined_point_list, point2idx = set(), {} # all points without grouping info [(x,y),...], points' index
    refined_group_points_list, refined_group_edges_info_list = [], [] # [[(x,y),(x2,y2),...], ... points in dif edge group]; # (points' index of group_edges, [0-6])
    ##### debug
    debug_group_edges_info_list = []
    debug_points_list, debug_point2idx = [], {} 
    poly = Polygon([(-2, -2), (W + 2, -2), (W + 2, H + 2), (-2, H + 2)])
    for i in range(len(group_points_list)):
        group_points = group_points_list[i]
        ##### debug
        #for j in range(len(group_points)):
        #    debug_points_list.append(group_points[j])
        ##### debug
        x_list, y_list = [], []
        for j in range(len(group_points)):
            x_list.append(group_points[j][0])
            y_list.append(group_points[j][1])
            if j < len(group_points) - 1:
                x_list.append((group_points[j][0] + group_points[j + 1][0]) / 2)
                y_list.append((group_points[j][1] + group_points[j + 1][1]) / 2)
        weights = [1 for _ in range(len(x_list))]
        weights[0] = 1e5
        weights[-1] = 1e5
        # spline interpolation, given a list of position and get a spline representation s: smoothing factor, k: degree of spline, w: weight
        # return, A tuple, (t,c,k) containing the vector of knots, the B-spline coefficients, and the degree of the spline.
        tck, _ = interp.splprep(
            [x_list, y_list], s=len(x_list), k=min(max(3, len(x_list) // 2), min(5, len(x_list) - 1)), w=weights
        )
        # evaluate a B-spline, which means generating the smoothed point list
        # np.linspace(range_start, range_end, number of points)
        xx, yy = interp.splev(np.linspace(0, 1, (len(group_points) - 1) * 3), tck, der=0)
        refined_group_points = (
            [(x_list[0], y_list[0])] + [(xx[k], yy[k]) for k in range(1, len(xx) - 1)] + [(x_list[-1], y_list[-1])]
        )
        for j in range(len(refined_group_points)):
            refined_point_list.add(refined_group_points[j])
        refined_group_points_list.append(refined_group_points)
        # Cutting Paths Area of the whole Polygon
        for j in range(len(refined_group_points) - 1):
            x1, x2, y1, y2 = (
                refined_group_points[j][0],
                refined_group_points[j + 1][0],
                refined_group_points[j][1],
                refined_group_points[j + 1][1],
            )
            normalx, normaly = y2 - y1, x1 - x2
            sqrtxy = sqrt(normalx**2 + normaly**2)
            normalx, normaly = 1e-4 * normalx / sqrtxy, 1e-4 * normaly / sqrtxy
            diff_poly = Polygon(
                [
                    (x1 + normalx, y1 + normaly),
                    (x1 - normalx, y1 - normaly),
                    (x2 - normalx, y2 - normaly),
                    (x2 + normalx, y2 + normaly),
                ]
            )
            poly = poly.difference(diff_poly) # Returns the part of poly that does not intersect with geometry diff_poly.
    refined_point_list = list(refined_point_list)
    for i, point in enumerate(refined_point_list):
        point2idx[point] = i
    for i in range(len(refined_group_points_list)):
        group_points = refined_group_points_list[i]
        group_edges = []
        for j in range(len(group_points) - 1):
            group_edges.append((point2idx[group_points[j]], point2idx[group_points[j + 1]]))
        refined_group_edges_info_list.append((group_edges, group_info_list[i])) # (points' index of group_edges, [0-6])
    ##### debug
    """     for i, point in enumerate(debug_points_list):
        debug_point2idx[point] = i
    for i in range(len(group_points_list)):
        group_points = group_points_list[i]
        group_edges = []
        for j in range(len(group_points) - 1):
            group_edges.append((debug_point2idx[group_points[j]], debug_point2idx[group_points[j + 1]]))
        debug_group_edges_info_list.append((group_edges, group_info_list[i])) # (points' index of group_edges, [0-6])
    """
    # decide areas
    areas_types_and_circuits = [] # (area_type 0-4, boundary_points' indexes)
    polys = [poly] if isinstance(poly, Polygon) else list(poly.geoms)
    for poly in polys:
        if isinstance(poly, Polygon):
            minx, miny, maxx, maxy = poly.bounds
            if minx < -1 or miny < -1 or maxx > W + 1 or maxy > H + 1:
                continue # the polygons covered buffer area
            centerx, centery = (minx + maxx) / 2, (miny + maxy) / 2
            mindis, minidx = 1e9, -1
            for i in range(len(area_centers_and_types)):
                dis = abs(area_centers_and_types[i][0] - centerx) + abs(area_centers_and_types[i][1] - centery)
                if dis < mindis:
                    mindis, minidx = dis, i
            if area_centers_and_types[minidx][2] == 0:  # unused
                continue
            circuit = [] # boundary points' indexes
            coords = list(poly.exterior.coords)[:-1] # get boundary points

            for p in coords:
                for point in refined_point_list:
                    if abs(point[0] - p[0]) + abs(point[1] - p[1]) < 5e-4:
                        if point not in circuit:
                            circuit.append(point2idx[point])
                        break
            areas_types_and_circuits.append((area_centers_and_types[minidx][2], circuit))
    """ 
    visualize_continuous(
        intersection_corners,
        debug_points_list,
        debug_group_edges_info_list,
        areas_types_and_circuits,
        color_set,
        "continuous_before_" + str(gen_idx),
    ) 
    """

    visualize_continuous(
        intersection_corners,
        refined_point_list,
        refined_group_edges_info_list,
        areas_types_and_circuits,
        color_set,
        "continuous_final_" + str(gen_idx),
    )
    return height_map, refined_point_list, refined_group_edges_info_list, areas_types_and_circuits


def single_gen(args: Tuple[dict, str, int, int, bool]) -> None:
    parameters, total_feedback, idx, gen_seed, vis = args
    random.seed(gen_seed)
    sys.stdout = open("logs/" + str(idx) + ".txt", "w")
    sys.stderr = open("logs/" + str(idx) + ".txt", "a")

    terrain = generate_terrain(parameters)
    infrastructure = generate_infrastructure(parameters, terrain)
    attributes = generate_attributes(parameters, terrain, infrastructure)
    if vis:
        visualize(
            terrain,
            attributes,
            infrastructure[0],
            [edge for edge in infrastructure[2] if infrastructure[2][edge] == 1],
            [edge for edge in infrastructure[2] if infrastructure[2][edge] == 2],
            infrastructure[1],
            color_set,
            terrain_label_set,
            content_marker_set,
            content_label_set,
            "coarse_final_" + str(idx),
        )
    np.save(
        "checkpoints/" + str(idx) + ".npy",
        np.array([parameters, total_feedback, terrain, infrastructure, attributes], dtype=object),
    )
    with open("data/data.json", "r") as dataf:
        data = json.load(dataf)
    with open("data/Gardenverse.json", "r") as dataf:
        gardenverse = json.load(dataf)
    
    height_map, points, edges, areas = make_continuous(terrain, attributes, infrastructure, idx)
    print("editing time")
    edit(height_map, points, edges, areas, infrastructure, color_set, data, idx)
    pcg(args, height_map, points, edges, areas, infrastructure, parameters, data, idx)


def single_gen_time(args: Tuple[dict, int, int]) -> None:
    parameters, idx, gen_seed = args
    random.seed(gen_seed)
    sys.stdout = open("logs/" + str(idx) + ".txt", "w")
    sys.stderr = open("logs/" + str(idx) + ".txt", "a")

    times = [time()]
    terrain = generate_terrain(parameters)
    times.append(time())
    infrastructure = generate_infrastructure(parameters, terrain)
    times.append(time())
    attributes = generate_attributes(parameters, terrain, infrastructure)
    times.append(time())

    with open("data/data.json", "r") as dataf:
        data = json.load(dataf)
        height_map, points, edges, areas = make_continuous(terrain, attributes, infrastructure, idx)
        times.append(time())
        pcg(args, height_map, points, edges, areas, infrastructure, parameters, data, idx)
        times.append(time())

    with open("ablation/time_" + str(W) + "_" + str(H) + "_" + str(idx) + ".txt", "w") as f:
        f.write(
            str(times[1] - times[0])
            + "\n"
            + str(times[2] - times[1])
            + "\n"
            + str(times[3] - times[2])
            + "\n"
            + str(times[4] - times[3])
            + "\n"
            + str(times[5] - times[4])
        )


def main(args, text: str, use_query: bool, ckp: str, num: int) -> None:
    print("using seed:", seed)
    terrain_sys_messages = form_sys_messages(
        terrain_sys_prompt,
        [terrain_sample_input1, terrain_sample_input2, terrain_sample_input3],
        [terrain_sample_output1, terrain_sample_output2, terrain_sample_output3],
    )
    inf_sys_messages = form_sys_messages(
        inf_sys_prompt,
        [inf_sample_input1, inf_sample_input2, inf_sample_input3],
        [inf_sample_output1, inf_sample_output2, inf_sample_output3],
    )
    if args.use_conlan:
        attribute_sys_messages = form_sys_messages(
            attribute_sys_prompt,
            [attribute_sample_input1, attribute_sample_input2, attribute_sample_input3],
            [attribute_sample_output1, attribute_sample_output2, attribute_sample_output3],
        )
    if num <= 1:
        print("single gen")
        random.seed(seed)
        parameters, total_feedback, terrain, infrastructure, attributes = [None for _ in range(5)]
        if os.path.exists("checkpoints/" + ckp + ".npy"):
            model = np.load("checkpoints/" + ckp + ".npy", allow_pickle=True).tolist()
            parameters, total_feedback, terrain, infrastructure, attributes = model
            print("load from " + ckp + ".npy")
            if args.regenerate_infrastructure:
                infrastructure = generate_infrastructure(args, parameters, terrain)
                visualize(
                    terrain,
                    [[0 for i in range(H)] for j in range(W)],
                    infrastructure[0],
                    [edge for edge in infrastructure[2] if infrastructure[2][edge] == 1],
                    [edge for edge in infrastructure[2] if infrastructure[2][edge] == 2],
                    infrastructure[1],
                    color_set,
                    terrain_label_set,
                    content_marker_set,
                    content_label_set,
                    "coarse_2",
                )
                np.save(
                    "checkpoints/" + ckp + ".npy",
                    np.array([parameters, total_feedback, terrain, infrastructure, attributes], dtype=object),
                )

            if args.regenerate_attributes:
                attributes = generate_attributes(args, parameters, terrain, infrastructure)
                visualize(
                    terrain,
                    attributes,
                    infrastructure[0],
                    [edge for edge in infrastructure[2] if infrastructure[2][edge] == 1],
                    [edge for edge in infrastructure[2] if infrastructure[2][edge] == 2],
                    infrastructure[1],
                    color_set,
                    terrain_label_set,
                    content_marker_set,
                    content_label_set,
                    "coarse_final_0",
                )
                np.save(
                    "checkpoints/" + ckp + ".npy",
                    np.array([parameters, total_feedback, terrain, infrastructure, attributes], dtype=object),
                )
        else:
            print("newly generate")
            terrain_gptres, inf_gptres, attribute_gptres = "", "", ""
            if use_query:
                threads = []
                threads.append(MyThread(query, (terrain_sys_messages, text))) # text is brief, so input same query many times
                threads.append(MyThread(query, (inf_sys_messages, text)))
                if args.use_conlan:
                    threads.append(MyThread(query, (attribute_sys_messages, text)))
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join()
                if args.use_conlan:
                    terrain_gptres, inf_gptres, attribute_gptres = (
                        threads[0].get_result(),
                        threads[1].get_result(),
                        threads[2].get_result(),
                    )
                else:
                    terrain_gptres, inf_gptres = (
                        threads[0].get_result(),
                        threads[1].get_result(),
                    )

            terrain_parameters, terrain_feedback = parse_terrain_gptres(terrain_gptres)
            inf_parameters, inf_feedback = parse_inf_gptres(inf_gptres)
            if not args.use_conlan:
                attribute_gptres = """{
                    "data":[
                        [1,-1,-1,-1,-1],
                        [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]],
                        [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]],
                        [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]]
                    ],
                    "feedback":"OK"
                }"""
            attribute_parameters, attribute_feedback = parse_attribute_gptres(attribute_gptres)

            parameters = {}
            parameters.update(terrain_parameters)
            parameters.update(inf_parameters)
            parameters.update(attribute_parameters)
            total_feedback = terrain_feedback + "\n" + inf_feedback + "\n" + attribute_feedback # "OK" in default
            print(parameters)

            terrain = generate_terrain(args, parameters) # grid
            visualize(
                terrain,
                [[0 for i in range(H)] for j in range(W)],
                [],
                [],
                [],
                [],
                color_set,
                terrain_label_set,
                content_marker_set,
                content_label_set,
                "coarse_1",
            )

            infrastructure = generate_infrastructure(args, parameters, terrain)
            visualize(
                terrain,
                [[0 for i in range(H)] for j in range(W)],
                infrastructure[0],
                [edge for edge in infrastructure[2] if infrastructure[2][edge] == 1],
                [edge for edge in infrastructure[2] if infrastructure[2][edge] == 2],
                infrastructure[1],
                color_set,
                terrain_label_set,
                content_marker_set,
                content_label_set,
                "coarse_2",
            )

            attributes = generate_attributes(args, parameters, terrain, infrastructure)
            visualize(
                terrain,
                attributes,
                infrastructure[0],
                [edge for edge in infrastructure[2] if infrastructure[2][edge] == 1],
                [edge for edge in infrastructure[2] if infrastructure[2][edge] == 2],
                infrastructure[1],
                color_set,
                terrain_label_set,
                content_marker_set,
                content_label_set,
                "coarse_final_0",
            )
            np.save(
                "checkpoints/" + ckp + ".npy",
                np.array([parameters, total_feedback, terrain, infrastructure, attributes], dtype=object),
            )

        with open("data/data.json", "r") as dataf:
            data = json.load(dataf)
        with open("data/GardenVerse.json", "r") as dataf:
            garden_verse = json.load(dataf)
        print("make continuous")
        height_map, points, edges, areas = make_continuous(terrain, attributes, infrastructure, 0)
        if args.edit:
            print("edit garden")
            edited_height_map, edited_points, edited_edges, edited_areas = edit(height_map, points, edges, areas, infrastructure, color_set, data, 0)
        if args.gen_edit_garden:
            print("pcg for edited garden")
            pcg(args, edited_height_map, edited_points, edited_edges, edited_areas, infrastructure, parameters, data, garden_verse, gen_idx=0, suffix = "edited", text=text)
        print("pcg for original garden")
        pcg(args, height_map, points, edges, areas, infrastructure, parameters, data, garden_verse, gen_idx=0, suffix="original", text=text)
        print("generation finished!")
    else:
        print("gen " + str(num) + " maps")
        parameters, total_feedback, terrain, infrastructure, attributes = [None for _ in range(5)]
        terrain_gptres, inf_gptres, attribute_gptres = "", "", ""
        if use_query:
            threads = []
            threads.append(MyThread(query, (terrain_sys_messages, text)))
            threads.append(MyThread(query, (inf_sys_messages, text)))
            threads.append(MyThread(query, (attribute_sys_messages, text)))
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            terrain_gptres, inf_gptres, attribute_gptres = (
                threads[0].get_result(),
                threads[1].get_result(),
                threads[2].get_result(),
            )

        terrain_parameters, terrain_feedback = parse_terrain_gptres(terrain_gptres)
        inf_parameters, inf_feedback = parse_inf_gptres(inf_gptres)
        attribute_parameters, attribute_feedback = parse_attribute_gptres(attribute_gptres)

        parameters = {}
        parameters.update(terrain_parameters)
        parameters.update(inf_parameters)
        parameters.update(attribute_parameters)
        total_feedback = terrain_feedback + "\n" + inf_feedback + "\n" + attribute_feedback
        print(parameters)

        with Pool(NUM_WORKERS) as p:
            r = list(
                tqdm(
                    p.imap(
                        single_gen,
                        [
                            (deepcopy(parameters), deepcopy(total_feedback), i, (i + 1) * seed, False)
                            for i in range(num)
                        ],
                    ),
                    total=num,
                )
            )


if __name__ == "__main__":
    if not os.path.exists("visuals"):
        os.mkdir("visuals")
    if not os.path.exists("outputs"):
        os.mkdir("outputs")
    if not os.path.exists("checkpoints"):
        os.mkdir("checkpoints")
    if not os.path.exists("logs"):
        os.mkdir("logs")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--text",
        "-t",
        type=str,
        default="A wonderful scape with mountains, water and forests, with almost no artificial traces.",
    )
    parser.add_argument(
        "--edit",
        "-e", 
        action="store_true"
    )
    parser.add_argument(
        "--gen_edit_garden",
        "-g", 
        action="store_true"
    )
    parser.add_argument(
        "--regenerate_infrastructure",
        "-i", 
        action="store_true"
    )
    parser.add_argument(
        "--regenerate_attributes",
        "-a", 
        action="store_true"
    )
    parser.add_argument(
        "--use_conlan",
        "-u", 
        action="store_true"
    )
    parser.add_argument(
        "--use_conlan_layout",
        "-l", 
        action="store_true"
    )

    parser.add_argument("--query", "-q", action="store_true")
    parser.add_argument("--seed", "-s", type=int, default=int(datetime.datetime.now().timestamp()))
    parser.add_argument("--checkpoint", "-c", type=str, default="0")
    parser.add_argument("--num", "-n", type=int, default=1)
    args = parser.parse_args()
    seed = args.seed
    main(args, args.text, args.query, args.checkpoint, args.num)
