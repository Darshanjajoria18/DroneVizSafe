# DroneSafePath

A Python-based drone deconfliction system for the FlytBase Robotics Assignment. Simulates 3D drone trajectories, detects conflicts within a 10m safety buffer, and visualizes results using interactive Plotly plots with constant trajectories, moving points, a 60-step time slider, a tracking camera, and red conflict markers. Supports four test scenarios (one no-conflict, three with conflicts). Outputs include HTML plots and MP4 animations. Implemented as a self-contained Jupyter notebook or modular codebase.

## Deliverables

### 1. Code Repository
- **Solution**: A Python solution simulating multiple drone trajectories and performing spatial-temporal conflict detection.
  - **Single Notebook Option**: `drone_deconfliction.ipynb` embeds test data (JSON strings) for portability, eliminating `FileNotFoundError` issues.
  - **Modular Option**: Files in `src/` (`data_loader.py`, `conflict_checker.py`, `visualization.py`, `main.py`) with test JSONs in `tests/`.
- **Modularity**: Code is organized into distinct modules:
  - Data loading and validation.
  - Conflict detection (spatial-temporal checks).
  - Visualization with Plotly.
- **Documentation**: Inline comments and docstrings explain functionality.
- **Test Scenarios**:
  - `no_conflict`: Clear paths.
  - `conflict_1`: Conflict at t≈12.5 (~8.66m).
  - `conflict_2`: Conflict at t≈5.0 (~7.07m).
  - `conflict_3`: Dual conflicts at t≈10 (~6.71m), t≈20 (~8.94m).

### 2. Documentation
- **README**: This file provides setup and execution instructions.
- **Reflection & Justification Document**: See `reflection.md` (1–2 pages) for:
  - Design decisions and architecture (modular vs. notebook).
  - Spatial-temporal conflict checks using linear interpolation and 30 time steps.
  - No AI integration; focus on deterministic algorithms.
  - Testing strategy: Four scenarios covering no-conflict, single, and dual conflicts; edge cases like simultaneous conflicts.
  - Scalability: Discusses real-world challenges (e.g., handling 10,000+ drones with optimized data structures, parallel processing).

## Setup Instructions

### Prerequisites
- Python 3.8+ (Anaconda recommended).
- VS Code with Python and Jupyter extensions (optional for notebook).
- Dependencies:
  ```bash
  pip install plotly numpy kaleido
  ```

### Project Structure
```
DroneSafePath/
├── src/                    # Modular code (optional)
│   ├── __init__.py
│   ├── data_loader.py
│   ├── conflict_checker.py
│   ├── visualization.py
│   ├── main.py
├── tests/                  # Test JSONs (for modular setup)
│   ├── test_data_no_conflict.json
│   ├── test_data_conflict_1.json
│   ├── test_data_conflict_2.json
│   ├── test_data_conflict_3.json
├── drone_deconfliction.ipynb  # Single notebook option
├── reflection.md           # Design and justification
├── README.md               # This file
├── .gitignore              # Ignores outputs, venvs
```

### Execution

#### Option 1: Jupyter Notebook (Recommended)
1. Open VS Code, select `C:\Users\Lenovo\OneDrive\Desktop\flytbase_assignment`.
2. Open `drone_deconfliction.ipynb`.
3. Select Python interpreter (Anaconda `base`).
4. Run all cells (`Ctrl+Alt+Shift+Enter`).
5. Last cell uses `test_name = 'conflict_3'`. Change to `'no_conflict'`, `'conflict_1'`, or `'conflict_2'` for other tests.
6. Outputs:
   - `trajectories_test_data_conflict_3.html`: Open in Chrome, use slider to view conflicts at t≈10, t≈20.
   - `trajectories_test_data_conflict_3.mp4`: ~8s animation.

#### Option 2: Modular Code
1. Ensure `tests/` contains JSONs.
2. In VS Code terminal:
   ```bash
   python src/main.py tests/test_data_conflict_3.json
   ```
3. Outputs: Same as above.

### Troubleshooting
- **FileNotFoundError**: Ensure `tests/` JSONs exist for modular setup. Notebook avoids this.
- **No MP4**: Use HTML if `kaleido` fails. Reinstall: `pip install --force-reinstall kaleido`.
- **No Red Markers**: Verify `conflict_checker.py` uses `num=30` time steps.

## Demo
- **Focus**: Run `conflict_3` to show dual conflicts (t≈10, ~6.71m; t≈20, ~8.94m).
- **Show**: Open `trajectories_test_data_conflict_3.html` in Chrome, slide to conflicts, highlight red markers, tracking camera, moving points.
- **Play**: `trajectories_test_data_conflict_3.mp4` with subtitles (e.g., via DaVinci Resolve).

## Submission
- **GitHub**: [https://github.com/<your-username>/DroneSafePath](https://github.com/<your-username>/DroneSafePath)
- **ZIP**: Exclude `.html`, `.mp4`:
  ```bash
  zip -r flytbase_assignment.zip . -x "*.html" -x "*.mp4"
  ```