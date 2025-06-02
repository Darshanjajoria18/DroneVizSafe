# Reflection & Justification: DroneSafePath

This document reflects on the design and implementation of the drone deconfliction system for the FlytBase Robotics Assignment, built in Python to simulate 3D drone trajectories and detect conflicts within a 10m safety buffer.

## Design Decisions & Architecture
I chose two formats: a Jupyter notebook (`drone_deconfliction.ipynb`) for simplicity and a modular setup (`src/` with `data_loader.py`, `conflict_checker.py`, `visualization.py`, `main.py`) for scalability. The notebook embeds test data as JSON strings, avoiding file dependency issues, while the modular code separates concerns (data, logic, visuals) for maintainability. Plotly was used for interactive 3D visualizations, showing constant trajectories, moving points, a 60-step slider, a tracking camera, and red conflict markers, making conflicts easy to spot.

## Spatial & Temporal Checks
The system interpolates drone positions linearly between waypoints using time steps (30 steps over the mission duration). For each step, it calculates Euclidean distances between the primary drone and others. If any distance is <10m, a conflict is flagged with time, distance, and drone ID. This approach ensures accurate temporal and spatial analysis without overcomplicating the logic.

## AI Integration
No AI was used. Deterministic interpolation and distance checks provide reliable results for small-scale tests. AI could optimize paths for large-scale scenarios but wasn’t needed here.

## Testing Strategy & Edge Cases
Four test cases were designed:
- No conflict: Safe paths.
- Conflict 1: Single conflict at t≈12.5 (~8.66m).
- Conflict 2: Conflict at t≈5 (~7.07m).
- Conflict 3: Dual conflicts at t≈10 (~6.71m), t≈20 (~8.94m).
Edge cases like simultaneous conflicts or near-boundary distances (≈10m) were tested. Outputs (HTML/MP4) were verified for correct red markers and slider functionality.

## Scalability
For 10,000+ drones, the current linear-time distance checks (O(n·t) per step) would be slow. I’d use spatial indexing (e.g., R-trees) to reduce checks to O(log n), parallel processing for time steps, and cloud platforms for compute. Data streaming and batch processing would handle real-time inputs.

## Conclusion
The system balances simplicity and functionality, with clear visualizations and robust tests. Scalability improvements would make it real-world ready.