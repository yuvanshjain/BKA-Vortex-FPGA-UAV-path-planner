# IMBKA-Vortex: Hardware-Accelerated UAV Path Planner 🚁⚡

**A full-stack hardware/software co-design implementing the Black-Winged Kite Algorithm (BKA) for real-time, obstacle-aware UAV path planning on the Zynq-7000 SoC.**

![Zynq-7000](https://img.shields.io/badge/Target-Zynq--7000%20SoC-blue) 

## 📌 Overview
This repository contains the high-level synthesis (HLS) C++ hardware engine and the interactive Python (PYNQ) overlay for a dynamic path-planning agent. It utilizes an Improved Metaheuristic Black-Winged Kite Algorithm (IMBKA) coupled with a custom vortex evasion protocol to navigate dense obstacle maps. 

The project demonstrates end-to-end System-on-Chip (SoC) integration, bridging complex mathematical algorithm design with strict hardware implementation constraints. By offloading swarm fitness evaluations and heavy trigonometry to programmable logic, this custom architecture achieves a **>5x execution speedup** over software-only implementations running on the ARM cortex—all while maintaining an incredibly small silicon footprint.

## 🚀 Key Hardware Optimizations
* **Hyper-Efficient Area Utilization:** Architected to be exceptionally lightweight, utilizing **only ~40% of available LUTs** and **< 50% of DSP slices** on the Zynq xc7z020 fabric. This massive area reduction was achieved by heavily recycling `square_distance` operators and avoiding unrolled evaluation loops, leaving abundant FPGA real estate for secondary IP cores.
* **Massive Hardware Acceleration:** Delivers a 5x+ performance increase over software execution by aggressively pipelining the core BKA math and isolating trigonometric functions to 24-bit precision.
* **Timing Closure:** Sequentially scheduled and resource-shared at a **50 MHz (20ns)** clock period without relying on heavy floating-point extensions (`fpext`) in the critical path.
* **Hybrid Evasion Logic:** Resolves the classic "always-left" local-minima bias by implementing a lightweight cross-product and goal-distance tie-breaker to calculate optimal, shortest-path corridors around obstacles.

## 💻 Interactive PYNQ UI
Features a Jupyter Notebook interface running on the Zynq Processing System (PS) for real-time map generation and AXI-Lite memory management.
* **Click and drag** to dynamically add custom obstacles and set their radius.
* **Press `Backspace`** to instantly undo/remove the last added obstacle.
* **Press `c`** to clear the entire map.
* **Press `Enter`** to trigger the hardware execution via AXI-Lite and generate the optimal flight trajectory.
  
  <img width="800" height="700" alt="image" src="https://github.com/user-attachments/assets/3244a55a-c18d-4abb-be77-89dbbfcf381b" />


## 🛠️ Tech Stack & VLSI Tools
* **Hardware Synthesis:** C++, Xilinx Vivado HLS
* **SoC Integration:** Vivado IP Integrator, AXI4 / AXI-Lite Interfaces
* **Target Board:** PYNQ-Z2 (Zynq-7000 ARM/FPGA SoC)
* **Software Overlay:** Python 3, PYNQ framework, NumPy, Matplotlib

## 📂 Repository Structure
* `/hls_source/` - The C++ source (`imbka_vortex.cpp`, `imbka_vortex_header.h`) and testbench files for Vivado HLS.
* `/pynq_overlay/` - The interactive Jupyter Notebook runner (`imbka_vortex_pynq.py`) along with the generated `.bit` and hardware handoff `.hwh` files.
* `/docs/` - A custom 13-page technical implementation guide and the original algorithmic reference paper.

## ⚙️ Getting Started
1. Clone this repository directly to your PYNQ-Z2 board's Jupyter workspace.
2. Ensure `imbkathree.bit` and `imbkathree.hwh` are located in the same directory as your execution script.
3. Open the Python script or Jupyter Notebook and run the UI block.
4. Interact with the matplotlib map to build your environment and press **Enter** to run the hardware accelerator.


### 📊 Data Extraction
The hardware engine saves the exact mathematical trajectory calculated during the run. If you need to access the raw data for external plotting, verification, or analysis, simply create a new cell below the UI and run:
```python
print(ui.path)
