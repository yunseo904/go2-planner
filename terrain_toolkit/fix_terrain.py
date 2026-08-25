"""numpy-only port of ``legged_gym/utils/terrain_utils.py::fix_terrain`` and
``calc_direct_path_heights``.

The upstream module cannot be imported on a CPU box because it pulls in
``pydelatin``, ``pyfqmr`` and ``torch`` at import time. The logic below is a
line-for-line transcription (only comments/formatting touched) so that the
frozen terrains match what ``Terrain.make_terrain`` would produce for the
``default``/custom terrain types.

NOTE: at the frozen upstream commit the ``benchmark`` branch of
``Terrain.make_terrain`` does **not** call ``fix_terrain``; we apply it here
because the planner project asked for the fixed state. Both the pre-fix and
post-fix height fields are stored in the frozen archive.
"""

from __future__ import annotations

import numpy as np


def fix_terrain(terrain) -> str:
    """Fix common errors with GPT-generated terrains (upstream port). Mutates ``terrain``."""
    # If goals are in units (indices), convert to meters
    env_length, env_width = terrain.width * terrain.horizontal_scale, terrain.length * terrain.horizontal_scale
    if np.max(terrain.goals[:, 0]) > env_length or np.max(terrain.goals[:, 1]) > env_width:
        terrain.goals = terrain.goals.astype(np.float64) * terrain.horizontal_scale

    fix_descs = set()

    min_terrain_height = np.min(terrain.height_field_raw)
    if min_terrain_height < round(-1 / terrain.vertical_scale):
        # NB: upstream compares against -1 (units), not -1 m. Kept verbatim.
        terrain.height_field_raw[terrain.height_field_raw < -1] = round(-1 / terrain.vertical_scale)
        fix_descs.add(f"min terrain height {min_terrain_height} is below -1")

    # Fix goals that are unset or out of bounds
    def valid_goal(goal):
        return 0 < goal[0] < env_length and 0 < goal[1] < env_width  # (0, 0) is the default

    num_goals_fixed = 0
    for i in range(1, len(terrain.goals)):
        if not valid_goal(terrain.goals[i]) and valid_goal(terrain.goals[i - 1]):
            terrain.goals[i] = terrain.goals[i - 1]
            num_goals_fixed += 1
    for i in range(len(terrain.goals) - 2, -1, -1):
        if not valid_goal(terrain.goals[i]) and valid_goal(terrain.goals[i + 1]):
            terrain.goals[i] = terrain.goals[i + 1]
            num_goals_fixed += 1
    if num_goals_fixed > 0:
        fix_descs.add(f"{num_goals_fixed} goal(s) out of bounds")
    assert num_goals_fixed <= round(len(terrain.goals) / 2), f"Fixed too many goals ({num_goals_fixed})!"
    for i in range(len(terrain.goals)):
        assert valid_goal(terrain.goals[i]), \
            f"Goal {i} at ({terrain.goals[i, 0]}, {terrain.goals[i, 1]}) is invalid!"

    # Move goals away from edge
    clipped_goals_x = np.clip(terrain.goals[:, 0], a_min=0.5, a_max=(env_length - 0.5))
    clipped_goals_y = np.clip(terrain.goals[:, 1], a_min=0.5, a_max=(env_width - 0.5))
    if not np.allclose(clipped_goals_x, terrain.goals[:, 0]) or not np.allclose(clipped_goals_y, terrain.goals[:, 1]):
        fix_descs.add("goals too close to edge")
    terrain.goals[:, 0] = clipped_goals_x
    terrain.goals[:, 1] = clipped_goals_y

    # Check and fix quadruped's spawn location
    if np.max(terrain.height_field_raw[:round(2 / terrain.horizontal_scale), :]) > 0:
        terrain.height_field_raw[:round(2 / terrain.horizontal_scale), :] = 0
        fix_descs.add("spawn area not 0")
    clipped_goals_x = np.clip(terrain.goals[:, 0], a_min=1.5, a_max=None)  # Move goals ahead of spawn
    if not np.allclose(clipped_goals_x, terrain.goals[:, 0]):
        fix_descs.add("goals too close to spawn")
    terrain.goals[:, 0] = clipped_goals_x

    # Check and fix small obstacles that have an extreme aspect ratio
    # (axis-aligned bounding boxes of flood-filled connected components)
    min_terrain_height = np.min(terrain.height_field_raw)
    valid_ratio_threshold = 2
    min_obstacle_length, min_obstacle_width = 0.6 / terrain.horizontal_scale, 0.4 / terrain.horizontal_scale
    floodfill_dz_threshold = 1 / terrain.vertical_scale
    obstacles = {}
    floodfill = np.zeros_like(terrain.height_field_raw)
    hf = terrain.height_field_raw
    H, W = hf.shape

    def bfs(x, y, id):
        # Same traversal order as upstream (FIFO, neighbours (0,1),(0,-1),(1,0),(-1,0));
        # uses a deque + head index instead of list.pop(0) for speed (order-preserving).
        q = [(x, y)]
        head = 0
        while head < len(q):
            x, y = q[head]
            head += 1
            if floodfill[x, y] != 0:
                continue
            floodfill[x, y] = id
            obstacles[id] = [
                (min(obstacles[id][0][0], x), min(obstacles[id][0][1], y)),
                (max(obstacles[id][1][0], x + 1), max(obstacles[id][1][1], y + 1)),
            ]
            hxy = hf[x, y]
            for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < H and 0 <= ny < W:
                    if hf[nx, ny] != min_terrain_height and floodfill[nx, ny] == 0 \
                            and abs(int(hf[nx, ny]) - int(hxy)) < floodfill_dz_threshold:
                        q.append((nx, ny))

    obstacle_counter = 0
    for i in range(H):
        for j in range(W):
            if hf[i, j] != min_terrain_height and floodfill[i, j] == 0:
                obstacle_counter += 1
                obstacles[obstacle_counter] = [(i, j), (i, j)]
                bfs(i, j, obstacle_counter)

    for obstacle in obstacles:
        x1, y1 = obstacles[obstacle][0]
        x2, y2 = obstacles[obstacle][1]
        obstacle_length, obstacle_width = x2 - x1, y2 - y1
        if max(obstacle_length, obstacle_width) / min(obstacle_width, obstacle_length) < valid_ratio_threshold:
            continue

        if obstacle_length < min_obstacle_length and obstacle_width < min_obstacle_width:
            # Erase small obstacles
            terrain.height_field_raw[x1:x2, y1:y2] = 0
            fix_descs.add("obstacles length and width too small (erased)")
        if obstacle_length < min_obstacle_length:
            # Extend length on both sides
            extend_length = max(round((min_obstacle_length - obstacle_length) // 2), 1)
            nx1, nx2 = max(0, x1 - extend_length), min(terrain.height_field_raw.shape[0], x2 + extend_length)
            terrain.height_field_raw[nx1:x1, y1:y2] = terrain.height_field_raw[x1, y1:y2][None, :]
            terrain.height_field_raw[x2:nx2, y1:y2] = terrain.height_field_raw[x2 - 1, y1:y2][None, :]
            fix_descs.add("obstacles length too small")
        if obstacle_width < min_obstacle_width:
            # Extend width on both sides
            extend_width = max(round((min_obstacle_width - obstacle_width) // 2), 1)
            ny1, ny2 = max(0, y1 - extend_width), min(terrain.height_field_raw.shape[1], y2 + extend_width)
            terrain.height_field_raw[x1:x2, ny1:y1] = terrain.height_field_raw[x1:x2, y1][..., None]
            terrain.height_field_raw[x1:x2, y2:ny2] = terrain.height_field_raw[x1:x2, y2 - 1][..., None]
            fix_descs.add("obstacles width too small")

    return ", ".join(sorted(fix_descs))


def calc_direct_path_heights(height_field_raw, goals, skip_size):
    """Bresenham line heights between consecutive goals (upstream port). ``goals`` in cell indices."""
    all_line_heights = []
    all_skip_line_heights = []
    for i in range(len(goals) - 1):
        (goal_x, goal_y), (next_goal_x, next_goal_y) = goals[i], goals[i + 1]
        goal_x, goal_y, next_goal_x, next_goal_y = round(goal_x), round(goal_y), round(next_goal_x), round(next_goal_y)

        dx, dy = abs(next_goal_x - goal_x), abs(next_goal_y - goal_y)
        sx, sy = 1 if goal_x < next_goal_x else -1, 1 if goal_y < next_goal_y else -1
        err = dx - dy

        x, y = goal_x, goal_y
        line_heights = [height_field_raw[x, y]]
        while x != next_goal_x or y != next_goal_y:
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            line_heights.append(height_field_raw[x, y])
        line_heights = np.asarray(line_heights)
        all_line_heights.append(line_heights)

        j = 0
        skip_line_heights = []
        while j < len(line_heights) - 1:
            skip_line_heights.append(line_heights[j])
            k = min(j + skip_size + 1, len(line_heights))
            diff_along_range = line_heights[j + 1:k] - line_heights[j]
            diff_along_range = np.maximum.accumulate(diff_along_range)
            diff_along_range = np.abs(diff_along_range)
            min_diff_idx = np.argmin(diff_along_range)
            j += min_diff_idx + 1
        skip_line_heights.append(line_heights[-1])
        all_skip_line_heights.append(np.asarray(skip_line_heights))

    return all_line_heights, all_skip_line_heights
