#
# README: UAV-UGV Cooperative Mission Planning Solver

## Overview
This Python script implements a bilevel optimization framework for UAV-UGV cooperative mission planning with recharging. It solves scenarios where UAVs and UGVs collaborate to complete mission tasks, with UAVs performing tasks and UGVs providing recharging support. The script uses metaheuristic methods to optimize the mission planning process.

## Features
- **Metaheuristic Methods**: Supports `TABU_SEARCH`, `SIMULATED_ANNEALING`, and `GUIDED_LOCAL_SEARCH` for solving the optimization problem.
- **Scenario Types**:
  - **Type A**: UAV points are identical to UGV points (road network).
  - **Type B**: UAV points are scattered around UGV points within a specified radius.
- **Visualization**: Plots the scenario, subproblems, and task allocations.
- **Optimization**:
  - Minimum Set Cover Algorithm (MSC) for UGV refueling stops.
  - E-VRPTW (Electric Vehicle Routing Problem with Time Windows) for UAV mission planning.
- **Customizable Parameters**: Allows users to specify scenario scale, metaheuristic methods, solver time, and plotting options.

## Requirements
- Python 3.7 or higher
- Required Python libraries:
  - `numpy`
  - `matplotlib`
  - `pandas`
  - `networkx`
  - `ortools`
  - `termcolor`

Install the required libraries using:
```bash
pip install numpy matplotlib pandas networkx ortools termcolor
```

## Usage
Run the script from the command line with the following arguments:

```bash
python solver_heu.py [OPTIONS]
```

### Command-Line Arguments
| Argument         | Type    | Default               | Description                                                                 |
|-------------------|---------|-----------------------|-----------------------------------------------------------------------------|
| `--scale`         | `str`   | `SS`                 | Scenario scale: `SS` (small), `MS` (medium), `LS` (large).                 |
| `--methods`       | `str`   | `GUIDED_LOCAL_SEARCH` | Metaheuristic methods to use (e.g., `TABU_SEARCH`, `SIMULATED_ANNEALING`). |
| `--solver_time`   | `int`   | `15`                 | Solver time in seconds for solving E-VRPTW inside the UAV planner.         |
| `--folder`        | `str`   | scenarios          | Folder name to store/load scenario data.                                   |
| `--plotting`      | `bool`  | `True`               | Enable or disable plotting (`True` or `False`).                            |

### Example Commands
1. Run with default parameters:
   ```bash
   python solver_heu.py
   ```

2. Run with a specific metaheuristic and solver time:
   ```bash
   python solver_heu.py --methods "TABU_SEARCH" --solver_time 10
   ```

3. Run with medium-scale scenarios and disable plotting:
   ```bash
   python solver_heu.py --scale MS --plotting False
   ```

## Script Workflow
1. **Scenario Loading**:
   - Loads scenario data from a pickle file in the specified folder.
   - Generates UAV and UGV points based on the scenario type.

2. **Optimization**:
   - **UGV Refueling Stops**: Uses the Minimum Set Cover Algorithm to determine optimal refueling stops.
   - **UAV Mission Planning**: Solves the E-VRPTW problem using OR-Tools with the specified metaheuristic.

3. **Visualization**:
   - Plots the scenario, subproblems, and task allocations if plotting is enabled.

4. **Output**:
   - Prints the total mission time and details of each subproblem.

## Key Functions
- `run_scenario_with_metaheuristic(metaheuristic, folder_name, plotting, scale, solver_time)`: Main function to run the scenario solver.
- `Major_Replan`: Class implementing the optimization framework, including:
  - `CP_set_cover`: Determines UGV refueling stops.
  - `Allocation_function`: Allocates UAV mission points to UGV stops.
  - `UAV_planner`: Solves the E-VRPTW problem for UAVs.

## Output
- **Console Output**:
  - Total mission time.
  - Details of each subproblem, including UAV sortie times and dropped locations.
- **Plots** (if enabled):
  - Scenario overview.
  - Subproblem-specific task allocations and paths.









# 
# README: Scenario Generator for UAV-UGV Mission Planning

## Overview

The  scenarios_generator.py  script generates scenarios for UAV-UGV cooperative mission planning. It supports two types of scenarios:

-   **Type A**: UAV points are identical to UGV points (road network).
-   **Type B**: UAV points are scattered randomly around UGV points within a specified radius.

## Features

-   Customizable number of points, radius, and coordinate range.
-   Supports small (`SS`), medium (`MS`), and large (`LS`) scales.
-   Saves scenarios as pickle files for use in optimization scripts.

## Requirements

-   Python 3.7+
-   Libraries:  `numpy`,  `pickle`,  `os`

Install dependencies:

```bash pip  install  numpy
```

## Usage

Run the script with:

``` python  scenarios_generator.py  [OPTIONS]
```

### Options

| Argument         | Type    | Default               | Description                                                                 |
|-------------------|---------|-----------------------|-----------------------------------------------------------------------------|
| `--scale`         | `str`   | `SS`                 | Scenario scale: `SS` (small), `MS` (medium), `LS` (large).                 |
| `--type`          | `str`   | `A`                  | scenario type  |                                |
| `--plotting`      | `bool`  | `True`               | Enable or disable plotting (`True` or `False`).     



### Examples

1.  Generate a Type A scenario:
    
    python  scenarios_generator.py  --type  A
    
2.  Generate a Type B scenario with a custom radius:
    
    python  scenarios_generator.py  --type  B  --radius  1.0  --scale  MS
    

## Output

-   Saves the scenario as  `<scale>_scenario_data.pkl`  in the specified folder.