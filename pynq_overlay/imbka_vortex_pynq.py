from pynq import Overlay, allocate
import matplotlib.pyplot as plt
import numpy as np
import struct
import time


MAX_OBS = 20
MAX_ITER = 800
MAP_LIMIT = 500.0

REG_CTRL = 0x00
REG_START_X = 0x10
REG_START_Y = 0x18
REG_GOAL_X = 0x20
REG_GOAL_Y = 0x28
REG_OBS_DATA = 0x30
REG_NUM_OBSTACLES = 0x3C
REG_PATH_X = 0x44
REG_PATH_Y = 0x50
REG_ITERATIONS_USED = 0x5C
REG_GOAL_REACHED = 0x6C


def _float_to_u32(value):
    return struct.unpack("<I", struct.pack("<f", np.float32(value)))[0]


def _buffer_address(buf):
    return int(getattr(buf, "device_address", getattr(buf, "physical_address")))


def _write_u64(ip, offset, value):
    value = int(value)
    ip.write(offset, value & 0xFFFFFFFF)
    ip.write(offset + 4, (value >> 32) & 0xFFFFFFFF)


class IMBKATwoRunner:
    def __init__(self, bitfile="imbkatwo.bit", ip_name=None):
        self.overlay = Overlay(bitfile)
        if ip_name is None:
            ip_name = next(name for name in self.overlay.ip_dict if "bka" in name.lower())
        self.ip_name = ip_name
        self.ip = getattr(self.overlay, ip_name)

        self.obs_buf = allocate(shape=(MAX_OBS * 3,), dtype=np.float32)
        self.path_x = allocate(shape=(MAX_ITER,), dtype=np.float32)
        self.path_y = allocate(shape=(MAX_ITER,), dtype=np.float32)

        self.start = (30.0, 30.0)
        self.goal = (420.0, 420.0)
        self.obstacles = []
        self.last_result = None

    def close(self):
        for buf in (self.obs_buf, self.path_x, self.path_y):
            try:
                buf.close()
            except Exception:
                pass

    def clear_obstacles(self):
        self.obstacles = []

    def add_obstacle(self, x, y, radius):
        if len(self.obstacles) >= MAX_OBS:
            raise ValueError(f"Only {MAX_OBS} obstacles are supported by the HLS core.")
        x = float(np.clip(x, 0.0, MAP_LIMIT))
        y = float(np.clip(y, 0.0, MAP_LIMIT))
        radius = float(np.clip(radius, 1.0, 200.0))
        self.obstacles.append((x, y, radius))

    def undo_obstacle(self):
        if self.obstacles:
            self.obstacles.pop()

    def run(self, start=None, goal=None, obstacles=None, timeout_s=10.0):
        if start is not None:
            self.start = tuple(map(float, start))
        if goal is not None:
            self.goal = tuple(map(float, goal))
        if obstacles is not None:
            self.obstacles = [tuple(map(float, obs)) for obs in obstacles]

        if len(self.obstacles) > MAX_OBS:
            raise ValueError(f"Only {MAX_OBS} obstacles are supported by the HLS core.")

        self.obs_buf[:] = 0.0
        flat_obs = self.obs_buf.reshape((MAX_OBS, 3))
        for i, obs in enumerate(self.obstacles):
            flat_obs[i, :] = obs
        self.obs_buf.flush()

        self.ip.write(REG_START_X, _float_to_u32(self.start[0]))
        self.ip.write(REG_START_Y, _float_to_u32(self.start[1]))
        self.ip.write(REG_GOAL_X, _float_to_u32(self.goal[0]))
        self.ip.write(REG_GOAL_Y, _float_to_u32(self.goal[1]))
        _write_u64(self.ip, REG_OBS_DATA, _buffer_address(self.obs_buf))
        self.ip.write(REG_NUM_OBSTACLES, len(self.obstacles))
        _write_u64(self.ip, REG_PATH_X, _buffer_address(self.path_x))
        _write_u64(self.ip, REG_PATH_Y, _buffer_address(self.path_y))

        t0 = time.time()
        self.ip.write(REG_CTRL, 0x01)
        while (self.ip.read(REG_CTRL) & 0x2) == 0:
            if time.time() - t0 > timeout_s:
                raise TimeoutError("BKA hardware timed out. Check bit/hwh pairing and buffer addresses.")

        elapsed_ms = (time.time() - t0) * 1000.0
        self.path_x.invalidate()
        self.path_y.invalidate()

        iterations = int(np.clip(self.ip.read(REG_ITERATIONS_USED), 1, MAX_ITER))
        reached = bool(self.ip.read(REG_GOAL_REACHED))
        path = np.column_stack((
            np.array(self.path_x[:iterations], dtype=np.float32),
            np.array(self.path_y[:iterations], dtype=np.float32),
        ))

        diffs = np.diff(path, axis=0)
        path_length = float(np.sum(np.linalg.norm(diffs, axis=1))) if len(path) > 1 else 0.0
        final_error = float(np.linalg.norm(path[-1] - np.array(self.goal, dtype=np.float32)))

        self.last_result = {
            "path": path,
            "iterations": iterations,
            "goal_reached": reached,
            "elapsed_ms": elapsed_ms,
            "path_length": path_length,
            "final_error": final_error,
        }
        return self.last_result

    def plot(self, result=None, ax=None, title_prefix="FPGA IMBKA path"):
        if result is None:
            result = self.last_result
        if result is None:
            raise ValueError("Run the planner before plotting.")
        if ax is None:
            _, ax = plt.subplots(figsize=(8, 8))

        path = result["path"]
        ax.clear()
        ax.plot(path[:, 0], path[:, 1], "k-", linewidth=2.5, label="FPGA path")
        ax.scatter(*self.start, color="green", s=120, label="Start", zorder=5)
        ax.scatter(*self.goal, color="tab:blue", marker="*", s=180, label="Goal", zorder=5)
        for x, y, r in self.obstacles:
            ax.add_patch(plt.Circle((x, y), r, color="red", alpha=0.30))

        status = "SUCCESS" if result["goal_reached"] else "FAILED"
        ax.set_title(
            f"[{status}] Time: {result['elapsed_ms']:.2f} ms | "
            f"Iters: {result['iterations']} | Final error: {result['final_error']:.1f} px"
        )
        ax.set_xlim(0, MAP_LIMIT)
        ax.set_ylim(0, MAP_LIMIT)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X (pixels)")
        ax.set_ylabel("Y (pixels)")
        ax.legend(loc="best")
        return ax


class IMBKAInteractiveMap:
    def __init__(self, runner, obstacle_radius=45.0):
        self.runner = runner
        self.obstacle_radius = float(obstacle_radius)
        self.mode = "obstacle"
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.cid_click = self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.cid_key = self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._draw()
        print("Mouse: left click adds an obstacle. Keys: r run, u undo, c clear, + radius up, - radius down, s set start, g set goal.")

    def _draw(self):
        self.ax.clear()
        self.ax.scatter(*self.runner.start, color="green", s=120, label="Start")
        self.ax.scatter(*self.runner.goal, color="tab:blue", marker="*", s=180, label="Goal")
        for x, y, r in self.runner.obstacles:
            self.ax.add_patch(plt.Circle((x, y), r, color="red", alpha=0.30))
        if self.runner.last_result is not None:
            path = self.runner.last_result["path"]
            self.ax.plot(path[:, 0], path[:, 1], "k-", linewidth=2.5, label="FPGA path")
        self.ax.set_title(f"Mode: {self.mode} | New obstacle radius: {self.obstacle_radius:.1f}")
        self.ax.set_xlim(0, MAP_LIMIT)
        self.ax.set_ylim(0, MAP_LIMIT)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="best")
        self.fig.canvas.draw_idle()

    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        if self.mode == "start":
            self.runner.start = (float(event.xdata), float(event.ydata))
            self.mode = "obstacle"
        elif self.mode == "goal":
            self.runner.goal = (float(event.xdata), float(event.ydata))
            self.mode = "obstacle"
        else:
            self.runner.add_obstacle(event.xdata, event.ydata, self.obstacle_radius)
        self._draw()

    def _on_key(self, event):
        if event.key == "r":
            result = self.runner.run()
            print(
                f"[{'SUCCESS' if result['goal_reached'] else 'FAILED'}] "
                f"{result['elapsed_ms']:.2f} ms, {result['iterations']} iterations, "
                f"length {result['path_length']:.1f}, final error {result['final_error']:.1f}"
            )
        elif event.key == "u":
            self.runner.undo_obstacle()
        elif event.key == "c":
            self.runner.clear_obstacles()
            self.runner.last_result = None
        elif event.key in ("+", "="):
            self.obstacle_radius = min(200.0, self.obstacle_radius + 5.0)
        elif event.key in ("-", "_"):
            self.obstacle_radius = max(1.0, self.obstacle_radius - 5.0)
        elif event.key == "s":
            self.mode = "start"
        elif event.key == "g":
            self.mode = "goal"
        self._draw()


if __name__ == "__main__":
    runner = IMBKATwoRunner("imbkatwo.bit")
    runner.add_obstacle(55.0, 270.0, 95.0)
    runner.add_obstacle(55.0, 210.0, 42.0)
    runner.add_obstacle(150.0, 240.0, 64.0)
    result = runner.run(start=(30.0, 30.0), goal=(420.0, 420.0))
    runner.plot(result)
    plt.show()
