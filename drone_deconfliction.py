#!/usr/bin/env python
# coding: utf-8

# In[1]:


import json
import numpy as np
from typing import List, Dict, Tuple
import plotly.graph_objects as go
import plotly.io as pio
import sys
import os

# Detect if running in Jupyter
def is_jupyter():
    try:
        from IPython import get_ipython
        return get_ipython() is not None
    except:
        return False

# Data structures
Waypoint = Tuple[float, float, float]
Mission = Dict[str, any]

def load_test_data(file_path: str) -> Dict:
    """Load and validate test data from JSON file."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Error: {file_path} not found. Ensure the file is in the same directory as the script.")
    
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            print(f"Loaded {file_path}")
    except json.JSONDecodeError as e:
        raise ValueError(f"Error: Invalid JSON in {file_path}: {e}")
    
    # Validate required fields
    if not isinstance(data, dict):
        raise ValueError("Error: JSON root must be an object.")
    
    if "mission" not in data:
        raise ValueError("Error: 'mission' field missing in JSON.")
    if not isinstance(data["mission"], dict):
        raise ValueError("Error: 'mission' must be an object.")
    
    if "waypoints" not in data["mission"] or not isinstance(data["mission"]["waypoints"], list):
        raise ValueError("Error: 'mission.waypoints' must be a list.")
    if not data["mission"]["waypoints"]:
        raise ValueError("Error: 'mission.waypoints' cannot be empty.")
    
    if "time_window" not in data["mission"] or not isinstance(data["mission"]["time_window"], dict):
        raise ValueError("Error: 'mission.time_window' must be an object.")
    if "start" not in data["mission"]["time_window"] or "end" not in data["mission"]["time_window"]:
        raise ValueError("Error: 'mission.time_window' must have 'start' and 'end'.")
    
    if "schedules" not in data or not isinstance(data["schedules"], list):
        raise ValueError("Error: 'schedules' must be a list.")
    
    # Validate waypoints
    required_fields = {"x": float, "y": float, "z": float, "time": float}
    for wp in data["mission"]["waypoints"]:
        for field, ftype in required_fields.items():
            if field not in wp or not isinstance(wp[field], (int, float)):
                raise ValueError(f"Error: Mission waypoint missing valid '{field}' (numeric).")
    
    for schedule in data["schedules"]:
        if "drone_id" not in schedule or not isinstance(schedule["drone_id"], str):
            raise ValueError("Error: Schedule missing valid 'drone_id' (string).")
        if "waypoints" not in schedule or not isinstance(schedule["waypoints"], list):
            raise ValueError(f"Error: Schedule for {schedule.get('drone_id', 'unknown')} missing 'waypoints' list.")
        if not schedule["waypoints"]:
            raise ValueError(f"Error: Waypoints for {schedule.get('drone_id', 'unknown')} cannot be empty.")
        for wp in schedule["waypoints"]:
            for field, ftype in required_fields.items():
                if field not in wp or not isinstance(wp[field], (int, float)):
                    raise ValueError(f"Error: Waypoint for {schedule.get('drone_id', 'unknown')} missing valid '{field}' (numeric).")
    
    # Validate test scenarios if present
    if "test_scenarios" in data:
        if not isinstance(data["test_scenarios"], dict):
            raise ValueError("Error: 'test_scenarios' must be an object.")
        for scenario_id, scenario in data["test_scenarios"].items():
            if not isinstance(scenario, dict):
                raise ValueError(f"Error: Scenario {scenario_id} must be an object.")
            if "drone_id" not in scenario or not isinstance(scenario["drone_id"], str):
                raise ValueError(f"Error: Scenario {scenario_id} missing valid 'drone_id' (string).")
            if "expected" not in scenario or scenario["expected"] not in ["clear", "conflict detected"]:
                raise ValueError(f"Error: Scenario {scenario_id} missing valid 'expected' ('clear' or 'conflict detected').")
    
    return data

def closest_distance_between_segments(p1: Waypoint, p2: Waypoint, q1: Waypoint, q2: Waypoint) -> float:
    """Calculate the closest distance between two 3D line segments."""
    u = np.array(p2) - np.array(p1)
    v = np.array(q2) - np.array(q1)
    w = np.array(p1) - np.array(q1)
    
    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)
    
    D = a * c - b * b
    sc, tc = 0.0, 0.0
    
    if D < 1e-10:  # Segments are parallel
        sc = 0.0
        tc = d / b if b > c else e / c
    else:
        sc = (b * e - c * d) / D
        tc = (a * e - b * d) / D
    
    sc = max(0.0, min(1.0, sc))
    tc = max(0.0, min(1.0, tc))
    
    dP = w + (sc * u) - (tc * v)
    return np.linalg.norm(dP)

def interpolate_position(wp1: Dict, wp2: Dict, t: float) -> Waypoint:
    """Interpolate drone position between two waypoints at time t."""
    t1, t2 = wp1["time"], wp2["time"]
    if t1 == t2:
        return (wp1["x"], wp1["y"], wp1["z"])
    frac = (t - t1) / (t2 - t1)
    frac = max(0.0, min(1.0, frac))
    x = wp1["x"] + frac * (wp2["x"] - wp1["x"])
    y = wp1["y"] + frac * (wp2["y"] - wp1["y"])
    z = wp1["z"] + frac * (wp2["z"] - wp1["z"])
    return (x, y, z)

def check_spatial_conflict(mission: Mission, schedule: Mission, time_window: Dict, safety_buffer: float) -> List[Dict]:
    """Check for spatial conflicts, considering time window overlap."""
    conflicts = []
    schedule_start = min(wp["time"] for wp in schedule["waypoints"])
    schedule_end = max(wp["time"] for wp in schedule["waypoints"])
    t_start, t_end = time_window["start"], time_window["end"]
    
    if schedule_end < t_start or schedule_start > t_end:
        return conflicts
    
    for i in range(len(mission["waypoints"]) - 1):
        t1_m, t2_m = mission["waypoints"][i]["time"], mission["waypoints"][i+1]["time"]
        for j in range(len(schedule["waypoints"]) - 1):
            t1_s, t2_s = schedule["waypoints"][j]["time"], schedule["waypoints"][j+1]["time"]
            if max(t1_m, t1_s) <= min(t2_m, t2_s):
                p1 = (mission["waypoints"][i]["x"], mission["waypoints"][i]["y"], mission["waypoints"][i]["z"])
                p2 = (mission["waypoints"][i+1]["x"], mission["waypoints"][i+1]["y"], mission["waypoints"][i+1]["z"])
                q1 = (schedule["waypoints"][j]["x"], schedule["waypoints"][j]["y"], schedule["waypoints"][j]["z"])
                q2 = (schedule["waypoints"][j+1]["x"], schedule["waypoints"][j+1]["y"], schedule["waypoints"][j+1]["z"])
                dist = closest_distance_between_segments(p1, p2, q1, q2)
                if dist < safety_buffer:
                    conflicts.append({
                        "segment_mission": (i, i+1),
                        "segment_schedule": (j, j+1),
                        "distance": dist,
                        "location": ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2, (p1[2] + p2[2])/2),
                        "time_range": (max(t1_m, t1_s), min(t2_m, t2_s))
                    })
    return conflicts

def check_temporal_conflict(mission: Mission, schedule: Mission, time_window: Dict, safety_buffer: float) -> List[Dict]:
    """Check for temporal conflicts within mission time window."""
    conflicts = []
    t_start, t_end = time_window["start"], time_window["end"]
    schedule_start = min(wp["time"] for wp in schedule["waypoints"])
    schedule_end = max(wp["time"] for wp in schedule["waypoints"])
    
    if schedule_end < t_start or schedule_start > t_end:
        return conflicts
    
    time_steps = np.linspace(max(t_start, schedule_start), min(t_end, schedule_end), num=10)
    seen_positions = set()
    
    for t in time_steps:
        for i in range(len(mission["waypoints"]) - 1):
            if mission["waypoints"][i]["time"] <= t <= mission["waypoints"][i+1]["time"]:
                pos_m = interpolate_position(mission["waypoints"][i], mission["waypoints"][i+1], t)
                for j in range(len(schedule["waypoints"]) - 1):
                    if schedule["waypoints"][j]["time"] <= t <= schedule["waypoints"][j+1]["time"]:
                        pos_s = interpolate_position(schedule["waypoints"][j], schedule["waypoints"][j+1], t)
                        dist = np.linalg.norm(np.array(pos_m) - np.array(pos_s))
                        if dist < safety_buffer:
                            pos_key = (round(pos_m[0], 2), round(pos_m[1], 2), round(pos_m[2], 2), round(t, 2))
                            if pos_key not in seen_positions:
                                seen_positions.add(pos_key)
                                conflicts.append({
                                    "time": t,
                                    "location": pos_m,
                                    "distance": dist,
                                    "drone_id": schedule.get("drone_id", "unknown")
                                })
    return conflicts

def check_mission_safety(mission: Mission, schedules: List[Mission], safety_buffer: float = 50.0) -> Dict:
    """Check if primary mission is safe against simulated schedules."""
    conflicts = []
    for schedule in schedules:
        spatial_conflicts = check_spatial_conflict(mission, schedule, mission["time_window"], safety_buffer)
        temporal_conflicts = check_temporal_conflict(mission, schedule, mission["time_window"], safety_buffer)
        for sc in spatial_conflicts:
            sc["drone_id"] = schedule.get("drone_id", "unknown")
            sc["type"] = "spatial"
        for tc in temporal_conflicts:
            tc["type"] = "temporal"
        conflicts.extend(spatial_conflicts + temporal_conflicts)
    return {
        "status": "clear" if not conflicts else "conflict detected",
        "details": conflicts
    }

def visualize_mission(mission: Mission, schedules: List[Mission], conflicts: List[Dict], output_file: str = "trajectories.html"):
    """Visualize 3D drone trajectories with constant lines and moving points."""
    fig = go.Figure()
    
    # Plot constant static trajectories
    x = [wp["x"] for wp in mission["waypoints"]]
    y = [wp["y"] for wp in mission["waypoints"]]
    z = [wp["z"] for wp in mission["waypoints"]]
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode="lines+markers", name="Primary Drone Path",
                               line=dict(color="blue", width=5, dash="dash"),
                               marker=dict(size=5, color="blue"), opacity=0.4))
    
    colors = ["orange", "green", "purple", "cyan", "magenta", "yellow"]
    for idx, schedule in enumerate(schedules):
        x = [wp["x"] for wp in schedule["waypoints"]]
        y = [wp["y"] for wp in schedule["waypoints"]]
        z = [wp["z"] for wp in schedule["waypoints"]]
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode="lines+markers", name=f"{schedule['drone_id']} Path",
                                   line=dict(color=colors[idx % len(colors)], width=5, dash="dash"),
                                   marker=dict(size=5, color=colors[idx % len(colors)]), opacity=0.4))
    
    # Initial moving points
    t_start, t_end = mission["time_window"]["start"], mission["time_window"]["end"]
    time_steps = np.linspace(t_start, t_end, num=60)  # 60 steps
    initial_data = []
    
    # Primary drone initial position
    for i in range(len(mission["waypoints"]) - 1):
        if mission["waypoints"][i]["time"] <= t_start <= mission["waypoints"][i+1]["time"]:
            pos = interpolate_position(mission["waypoints"][i], mission["waypoints"][i+1], t_start)
            initial_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                             name="Primary Drone (Moving)",
                                             marker=dict(size=8, color="blue"),
                                             text=["Primary Drone"], hoverinfo="text"))
            break
        elif t_start < mission["waypoints"][0]["time"]:
            pos = (mission["waypoints"][0]["x"], mission["waypoints"][0]["y"], mission["waypoints"][0]["z"])
            initial_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                             name="Primary Drone (Moving)",
                                             marker=dict(size=8, color="blue"),
                                             text=["Primary Drone"], hoverinfo="text"))
            break
    
    # Other drones' initial positions
    for idx, schedule in enumerate(schedules):
        if not schedule["waypoints"]:
            continue
        start_idx = min(range(len(schedule["waypoints"])), key=lambda i: schedule["waypoints"][i]["time"])
        end_idx = max(range(len(schedule["waypoints"])), key=lambda i: schedule["waypoints"][i]["time"])
        t_start_schedule = schedule["waypoints"][start_idx]["time"]
        t_end_schedule = schedule["waypoints"][end_idx]["time"]
        if t_start <= t_end_schedule:
            for j in range(len(schedule["waypoints"]) - 1):
                if schedule["waypoints"][j]["time"] <= t_start <= schedule["waypoints"][j+1]["time"]:
                    pos = interpolate_position(schedule["waypoints"][j], schedule["waypoints"][j+1], t_start)
                    initial_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                                     name=f"{schedule['drone_id']} (Moving)",
                                                     marker=dict(size=6, color=colors[idx % len(colors)]),
                                                     text=[schedule["drone_id"]], hoverinfo="text"))
                    break
                elif t_start < schedule["waypoints"][0]["time"]:
                    pos = (schedule["waypoints"][0]["x"], schedule["waypoints"][0]["y"], schedule["waypoints"][0]["z"])
                    initial_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                                     name=f"{schedule['drone_id']} (Moving)",
                                                     marker=dict(size=6, color=colors[idx % len(colors)]),
                                                     text=[schedule["drone_id"]], hoverinfo="text"))
                    break
    
    # Initial conflict markers
    for conflict in conflicts:
        if conflict["type"] == "temporal" and abs(conflict["time"] - t_start) < 0.5:
            x, y, z = conflict["location"]
            initial_data.append(go.Scatter3d(x=[x], y=[y], z=[z], mode="markers+text",
                                             name=f"Conflict ({conflict['drone_id']})",
                                             marker=dict(size=10, color="red"),
                                             text=[f"{conflict['drone_id']} t={t_start:.1f}"],
                                             textposition="top center"))
    
    fig.add_traces(initial_data)
    
    # Animation frames
    frames = []
    for t in time_steps:
        frame_data = []
        # Primary drone
        for i in range(len(mission["waypoints"]) - 1):
            if mission["waypoints"][i]["time"] <= t <= mission["waypoints"][i+1]["time"]:
                pos = interpolate_position(mission["waypoints"][i], mission["waypoints"][i+1], t)
                frame_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                               name="Primary Drone (Moving)",
                                               marker=dict(size=8, color="blue"),
                                               text=["Primary Drone"], hoverinfo="text"))
                break
            elif t < mission["waypoints"][0]["time"]:
                pos = (mission["waypoints"][0]["x"], mission["waypoints"][0]["y"], mission["waypoints"][0]["z"])
                frame_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                               name="Primary Drone (Moving)",
                                               marker=dict(size=8, color="blue"),
                                               text=["Primary Drone"], hoverinfo="text"))
                break
        
        # Other drones
        for idx, schedule in enumerate(schedules):
            if not schedule["waypoints"]:
                continue
            start_idx = min(range(len(schedule["waypoints"])), key=lambda i: schedule["waypoints"][i]["time"])
            end_idx = max(range(len(schedule["waypoints"])), key=lambda i: schedule["waypoints"][i]["time"])
            t_start_schedule = schedule["waypoints"][start_idx]["time"]
            t_end_schedule = schedule["waypoints"][end_idx]["time"]
            if t_start_schedule <= t <= t_end_schedule:
                for j in range(len(schedule["waypoints"]) - 1):
                    if schedule["waypoints"][j]["time"] <= t <= schedule["waypoints"][j+1]["time"]:
                        pos = interpolate_position(schedule["waypoints"][j], schedule["waypoints"][j+1], t)
                        frame_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                                       name=f"{schedule['drone_id']} (Moving)",
                                                       marker=dict(size=6, color=colors[idx % len(colors)]),
                                                       text=[schedule["drone_id"]], hoverinfo="text"))
                        break
                    elif t < schedule["waypoints"][0]["time"]:
                        pos = (schedule["waypoints"][0]["x"], schedule["waypoints"][0]["y"], schedule["waypoints"][0]["z"])
                        frame_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], mode="markers",
                                                       name=f"{schedule['drone_id']} (Moving)",
                                                       marker=dict(size=6, color=colors[idx % len(colors)]),
                                                       text=[schedule["drone_id"]], hoverinfo="text"))
                        break
        
        # Conflicts
        for conflict in conflicts:
            if conflict["type"] == "temporal" and abs(conflict["time"] - t) < 0.5:
                x, y, z = conflict["location"]
                frame_data.append(go.Scatter3d(x=[x], y=[y], z=[z], mode="markers+text",
                                               name=f"Conflict ({conflict['drone_id']})",
                                               marker=dict(size=10, color="red"),
                                               text=[f"{conflict['drone_id']} t={t:.1f}"],
                                               textposition="top center"))
        
        frames.append(go.Frame(data=frame_data, name=f"t={t:.1f}"))
    
    fig.frames = frames
    
    # Add camera motion
    camera_frames = []
    for t in time_steps:
        for i in range(len(mission["waypoints"]) - 1):
            if mission["waypoints"][i]["time"] <= t <= mission["waypoints"][i+1]["time"]:
                pos = interpolate_position(mission["waypoints"][i], mission["waypoints"][i+1], t)
                camera_frames.append(dict(eye=dict(x=pos[0]+70, y=pos[1]+70, z=pos[2]+50)))
                break
        else:
            pos = (mission["waypoints"][0]["x"], mission["waypoints"][0]["y"], mission["waypoints"][0]["z"])
            camera_frames.append(dict(eye=dict(x=pos[0]+70, y=pos[1]+70, z=pos[2]+50)))
    for i, frame in enumerate(frames):
        frame["layout"] = {"scene_camera": camera_frames[i]}
    
    # Layout with fixed slider
    fig.update_layout(
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="auto", camera=dict(eye=dict(x=2.0, y=2.0, z=1.5))
        ),
        title="Drone Simulation: Constant Trajectories",
        updatemenus=[dict(
            type="buttons",
            showactive=False,
            buttons=[
                dict(label="Play",
                     method="animate",
                     args=[None, dict(frame=dict(duration=75, redraw=True),
                                      fromcurrent=True, mode="immediate")]),
                dict(label="Pause",
                     method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate")])
            ]
        )],
        sliders=[dict(
            steps=[dict(method="animate",
                        args=[[f"t={t:.1f}"],
                              dict(mode="immediate", frame=dict(duration=75, redraw=True))],
                        label=f"t={t:.1f}") for t in time_steps],
            active=0,
            currentvalue={"prefix": "Time: "},
            pad={"t": 50}
        )],
        showlegend=True,
        legend=dict(x=0.1, y=0.9)
    )
    
    # Save and display
    try:
        fig.write_html(output_file)
        print(f"Saved visualization to {output_file}")
    except Exception as e:
        print(f"Failed to save HTML: {e}")
    
    try:
        fig.show()
    except Exception as e:
        print(f"Failed to display plot: {e}. Open {output_file} in Chrome/Firefox.")
    
    # Export video
    try:
        video_file = f"trajectories_{os.path.splitext(os.path.basename(output_file))[0]}.mp4"
        pio.write_image(fig, file=video_file, format="mp4", engine="kaleido",
                        animation_frame="name", animation_group="name")
        print(f"Video exported as {video_file}")
    except Exception as e:
        print(f"Video export failed: {e}. Install kaleido: `pip install kaleido`")

def run_tests(data: Dict):
    """Run tests using test scenarios."""
    mission = data["mission"]
    schedules = data["schedules"]
    test_scenarios = data.get("test_scenarios", {})
    for scenario_id, scenario in test_scenarios.items():
        drone_id = scenario["drone_id"]
        schedule = next((s for s in schedules if s["drone_id"] == drone_id), None)
        if schedule is None:
            print(f"Scenario {scenario_id}: Error - Drone {drone_id} not found in schedules")
            continue
        result = check_mission_safety(mission, [schedule])
        print(f"Scenario {scenario_id}: {scenario['description']}")
        print(f"Expected: {scenario['expected']}, Got: {result['status']}")
        print(f"Details: {result['details']}\n")

if __name__ == "__main__":
    # Default file
    data_file = "test_data.json"
    
    # Handle arguments
    if is_jupyter():
        # In Jupyter, ignore sys.argv and use default unless explicitly set
        print("Running in Jupyter. Using default file: {}".format(data_file))
    else:
        # In CLI, check for valid file argument
        if len(sys.argv) >= 2:
            # Filter out invalid arguments (e.g., '-f')
            valid_args = [arg for arg in sys.argv[1:] if arg.endswith('.json') and not arg.startswith('-')]
            if valid_args:
                data_file = valid_args[0]
                if len(valid_args) > 1:
                    print("Warning: Multiple JSON files provided. Using only: {}".format(data_file))
            else:
                print("Warning: No valid JSON file provided. Using default: {}".format(data_file))
    
    try:
        data = load_test_data(data_file)
        mission = data["mission"]
        schedules = data["schedules"]
        result = check_mission_safety(mission, schedules)
        print("Mission Safety Check Result:", result)
        output_base = os.path.splitext(os.path.basename(data_file))[0]
        output_file = f"trajectories_{output_base}.html"
        visualize_mission(mission, schedules, result["details"], output_file=output_file)
        run_tests(data)
    except (FileNotFoundError, ValueError) as e:
        print(e)
        sys.exit(1)


# In[ ]:




