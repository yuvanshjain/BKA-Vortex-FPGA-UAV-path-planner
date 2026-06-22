# IMBKA-Vortex: Hardware-Accelerated UAV Path Planner 🚁⚡

**A full-stack hardware/software co-design implementing the Black-Winged Kite Algorithm (BKA) for real-time, obstacle-aware UAV path planning on the Zynq-7000 SoC.**

![Zynq-7000](https://img.shields.io/badge/Target-Zynq--7000%20SoC-blue) ![Speedup](https://img.shields.io/badge/Acceleration->5x%20vs%20Raspberry%20Pi-success) ![LUTs](https://img.shields.io/badge/LUT_Utilization-~40%25-brightgreen) ![DSPs](https://img.shields.io/badge/DSP_Utilization-%3C50%25-brightgreen) 

## 📌 Overview
This repository contains the high-level synthesis (HLS) C++ hardware engine and the interactive Python (PYNQ) overlay for a dynamic path-planning agent. It utilizes an Improved Metaheuristic Black-Winged Kite Algorithm (IMBKA) coupled with a custom vortex evasion protocol to navigate dense obstacle maps. 

The project demonstrates end-to-end SoC integration, bridging complex mathematical algorithm design with strict hardware implementation constraints. By offloading swarm fitness evaluations and heavy trigonometry to programmable logic, this custom architecture operates **over 5x faster than a Raspberry Pi** running the software-equivalent model.

## 🚀 True Hardware Architecture & Optimizations
* **Sequential Execution & Area Reduction:** To fit within the strict area budget of the Zynq xc7z020 fabric, loop unrolling and pipelining were explicitly disabled (`PIPELINE off`, `unroll factor=1`). By forcing sequential execution, the multiplexer footprint was drastically reduced, resulting in a hyper-lightweight core using only ~40% of available LUTs.
* **Native Fixed-Point Math:** Floating-point extensions (`fpext`) were eradicated from the critical path. The design utilizes native `ap_ufixed` types and `hls::sqrt` routines, isolating trigonometric functions to 24-bit precision to prevent overflow and save DSP slices.
* **Map-Aware Vortex Evasion:** Replaces the standard local left/right obstacle vote with a map-aware corridor cost logic. On vortex entry, the planner samples tangential corridors against all obstacles, boundaries, and the goal distance to calculate the lowest-cost route, resolving the classic "always-left" local minima bias.

## 💻 Interactive PYNQ UI
Features a Jupyter Notebook interface running on the Zynq Processing System (PS) utilizing direct AXI-Lite memory management. 

Interact with the matplotlib map to build your environment:
* **Click and drag** to add custom obstacles.
* **Press `+` or `-`** to adjust the radius of the next obstacle.
* **Press `u`** to undo/remove the last added obstacle.
* **Press `s` or `g`** to enter Start Point or Goal Point placement mode.
* **Press `c`** to clear the entire map.
* **Press `r`** to trigger the hardware execution via AXI-Lite and generate the trajectory.

<p align="center">
  <br><br>
  <img width="800" height="700" alt="image" src="https://github.com/user-attachments/assets/3244a55a-c18d-4abb-be77-89dbbfcf381b" />
</p>

### 📊 Data Extraction
The hardware engine saves the exact mathematical trajectory calculated during the run. To access the raw data array for external plotting or verification, create a new cell in Jupyter and run:
```python
print(ui.path)
