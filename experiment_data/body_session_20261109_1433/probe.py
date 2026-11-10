"""Supervised local-only live probe; never connects to a hardware transport."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import robots  # noqa: F401
from core.app import operator_control
from core.app.handlers import teleop
from core.registry import ROBOT_REGISTRY
from gmr_body import create_backend
from tests.integration.test_body_gmr_pipeline import make_client
from transport.zmq import ZmqTransport

robot = ROBOT_REGISTRY.build("gento_luna")
config = SimpleNamespace(transport=SimpleNamespace(
    sub_endpoint="tcp://127.0.0.1:15555", pub_endpoint="tcp://127.0.0.1:15556",
    disabled_cameras=[], disabled_groups=[],
))
transport = ZmqTransport(config, robot)
backend = create_backend()
backend.config.update(allow_unvalidated_output=True, relative_task_alignment=True,
                      relative_position_scale=0.8)
client = make_client()
client._backend = backend
client.start()
state = teleop.TeleopExecutionState(active=False)
runtime = SimpleNamespace(robot=robot, transport=transport, teleop_client=client,
                          teleop_execution=state, rl_active=False,
                          collection_teleop_armed=False, collection_teleop_active=False)
session = SimpleNamespace(last_error="")
path = Path("work_dirs") / ("live_body_sim_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
ever_armed = False

def dispatch(command, _config, _runtime, _session):
    global ever_armed
    if command == "web:collect_arm:on":
        client.reset(require_neutral=False)
        state.active = True
        runtime.collection_teleop_armed = runtime.collection_teleop_active = True
        ever_armed = True
        print("ARM ON: waiting for both grip latches", flush=True)
    elif command == "web:collect_arm:off":
        state.active = False
        runtime.collection_teleop_armed = runtime.collection_teleop_active = False
        print("ARM OFF", flush=True)
    else:
        raise ValueError(f"unsupported probe command {command}")

try:
    deadline = time.monotonic() + 180
    last_report = 0.0
    commands = 0
    print("READY: local simulation only; ARM is OFF", flush=True)
    with path.open("w", encoding="utf8") as log:
        while time.monotonic() < deadline:
            for event in client.drain_events():
                if event.intent == "arm_toggle":
                    operator_control.handle_teleop_operator_event(event, config, runtime, session, dispatch=dispatch)
                else:
                    client.acknowledge_event(event, accepted=False, message="Not used by local simulation probe")
            if ever_armed and not runtime.collection_teleop_armed:
                break
            if state.active:
                capture = {}
                try:
                    published = teleop._step_active_teleop(None, runtime, state, client, time.monotonic(), capture=capture)
                except Exception as error:
                    print("FAULT", type(error).__name__, str(error), flush=True)
                    break
                commands += int(published is not None)
                log.write(json.dumps({"time": time.monotonic(), "kind": capture.get("result_kind"),
                                      "fault": state.last_fault, "source": capture.get("source"),
                                      "published": published is not None,
                                      "feedback_qpos": capture["robot"]["qpos"].tolist(),
                                      "published_qpos": None if published is None else published.qpos.tolist()}) + "\n")
                log.flush()
                if state.last_fault:
                    print("FAULT", state.last_fault, flush=True)
                    break
            now = time.monotonic()
            if now - last_report >= 5:
                status = client.status()
                print("STATUS", "connected", status.connected, "armed", runtime.collection_teleop_armed,
                      "authorized", status.authorized_groups, "commands", commands, flush=True)
                last_report = now
            time.sleep(0.05)
    print("END", "commands", commands, "log", str(path), flush=True)
finally:
    state.active = False
    client.close()
    transport.close()
