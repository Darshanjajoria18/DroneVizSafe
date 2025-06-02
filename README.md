# DroneSafePath

A Python-based drone deconfliction system for the FlytBase Robotics Assignment. Simulates 3D drone trajectories, detects conflicts within a 10m safety buffer, and visualizes results using interactive Plotly plots with constant trajectories, moving points, a 60-step time slider, a tracking camera, and red conflict markers. Supports four test scenarios (one no-conflict, three with conflicts). Outputs include HTML plots and MP4 animations. Implemented as a self-contained Jupyter notebook or modular codebase.

## Deliverables

### 1. Code Repository
- **Solution**: A Python solution simulating multiple drone trajectories and performing spatial-temporal conflict detection.
  - **Single Notebook Option**: `drone_deconfliction.ipynb` embeds test data (JSON strings) for portability, eliminating `FileNotFoundError` issues.
- **Modularity**: Code is organized into distinct modules:
  - Data loading and validation.
  - Conflict detection (spatial-temporal checks).
  - Visualization with Plotly.
- **Documentation**: Inline comments and docstrings explain functionality.
- **Test Scenarios**:
  - `no_conflict`: Clear paths.
  - `conflict_1`: Conflict at t≈12.5 (~8.66m).
  - `conflict_2`: Dual conflicts at t≈10 (~6.71m), t≈20 (~8.94m).
    
## Setup Instructions

### Prerequisites
- Python 3.8+ (Anaconda recommended).
- VS Code with Python and Jupyter extensions (optional for notebook).
- Dependencies:
  ```bash
  pip install plotly numpy kaleido
