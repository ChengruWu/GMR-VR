"""Stream a read-only third-person MuJoCo view to the EVA-VR Body dashboard.

Use only with a local simulation observation endpoint. No robot commands are sent.
The 2-D JPEG panel is independent of the WebXR geometry stream.
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import sys
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
import zmq
from websockets.legacy.server import serve

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
import robots  # noqa: E402,F401
from core.registry import ROBOT_REGISTRY  # noqa: E402
from transport.zmq import unpack_observation  # noqa: E402


async def main(args):
    model = mujoco.MjModel.from_xml_path(str(ROOT / "examples/hardware/gento_luna/scenes/box_task.xml"))
    data = mujoco.MjData(model)
    robot = ROBOT_REGISTRY.build("gento_luna")
    groups = {g.name: [int(model.jnt_qposadr[model.joint(name).id]) for name in g.joint_names]
              for g in robot.actuator_groups}
    base = [int(model.jnt_qposadr[model.joint(name).id]) for name in ("base_x", "base_y", "base_yaw")]
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0, 0, 0.9)
    camera.distance, camera.azimuth, camera.elevation = 3.0, 135.0, -15.0
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.connect(args.obs_endpoint)
    clients = set()

    async def handler(socket, path):
        if path != "/view":
            await socket.close(code=1008, reason="unknown path")
            return
        clients.add(socket)
        try:
            await socket.wait_closed()
        finally:
            clients.discard(socket)

    async def render_loop():
        last_received = 0.0
        while True:
            await asyncio.sleep(1 / args.fps)
            try:
                wire = unpack_observation(sub.recv(zmq.NOBLOCK))
            except zmq.Again:
                wire = None
            if wire is not None:
                for name, addresses in groups.items():
                    values = wire.state.get(name)
                    if values is None or len(values) != len(addresses) or not np.isfinite(values).all():
                        break
                else:
                    for name, addresses in groups.items():
                        data.qpos[addresses] = wire.state[name]
                    if wire.base_pose is not None and np.isfinite(wire.base_pose).all():
                        data.qpos[base] = wire.base_pose
                    last_received = time.monotonic()
            if not clients or time.monotonic() - last_received > 1.0:
                continue
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            rgb = renderer.render()
            ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                    [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                continue
            payload = jpeg.tobytes()
            for socket in tuple(clients):
                try:
                    await asyncio.wait_for(socket.send(payload), timeout=0.25)
                except (asyncio.TimeoutError, Exception):
                    await socket.close()

    try:
        async with serve(handler, "127.0.0.1", args.port, compression=None, max_queue=1):
            print(f"Simulation panel ready on 127.0.0.1:{args.port}; observation {args.obs_endpoint}", flush=True)
            await render_loop()
    finally:
        sub.close(linger=0)
        context.term()
        renderer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--obs-endpoint", default="tcp://127.0.0.1:15555")
    parser.add_argument("--port", type=int, default=43877)
    parser.add_argument("--fps", type=float, default=8)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    args = parser.parse_args()
    if not 1 <= args.fps <= 15 or not 1 <= args.width <= 1280 or not 1 <= args.height <= 720:
        parser.error("fps must be 1..15 and dimensions at most 1280x720")
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main(args))
