#!/usr/bin/env python3
"""SG2 L-table VR recording launcher dashboard (stdlib only)."""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
CYCLO = ROOT / "cyclo_lab"
VR_REPO = ROOT / "robotis_applications"
AI_REPO = ROOT / "ai_worker"
PORT = int(os.environ.get("DASHBOARD_PORT", "8765"))
TELEOP_GRIP_STATUS = Path(
    os.environ.get("EYKOREA_TELEOP_GRIP_STATUS", "/tmp/eykorea_teleop_grip.json")
)
ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "30")
RMW = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
# Selectable tasks. Each maps a display label to its gym id, dataset file, and
# robot profile (gripper SG2 vs dexterous-hand SH5).
ROBOT_PROFILES: dict[str, dict[str, str]] = {
    "FFW_SG2": {
        "robot_type": "FFW_SG2",
        "vr_model": "sg2",
        "hand": "false",
        # SG2 lift is controlled by the right thumbstick in the VR publisher.
        "vr_extra_args": "",
        "urdf": (
            "/root/ros2_ws/install/ffw_description/share/ffw_description/urdf/"
            "ffw_sg2_rev1_follower/ffw_sg2_follower.urdf"
        ),
    },
    "FFW_SH5": {
        "robot_type": "FFW_SH5",
        "vr_model": "sh5",
        "hand": "true",
        # SH5 lift is LOCKED for hands tasks: VR does not publish lift commands,
        # so the lift holds the per-task reset height. Head publishing stays OFF
        # so the head camera stays on the task.
        "vr_extra_args": "enable_lift_publishing:=false",
        "urdf": (
            "/root/ros2_ws/install/ffw_description/share/ffw_description/urdf/"
            "ffw_sh5_rev1_follower/ffw_sh5_follower.urdf"
        ),
    },
}
LIFT_ENABLED = os.environ.get("SH5_ENABLE_LIFT", "1") not in ("0", "false", "False")
# Stereo passthrough image shown as the VR background. Off by default so the
# headset shows a clean scene instead of the camera image. Set VR_IMAGE=1 to
# re-enable the passthrough background.
VR_IMAGE_ENABLED = os.environ.get("VR_IMAGE", "0") not in ("0", "false", "False")

TASKS: dict[str, dict[str, str]] = {
    "Basket Pick & Place (SG2)": {
        "id": "Cyclo-Real-Pick-Place-FFW-SG2-v0",
        "mimic_id": "Cyclo-Real-Mimic-Pick-Place-FFW-SG2-v0",
        "dataset": "ffw_sg2_basket_raw.hdf5",
        "robot": "FFW_SG2",
    },
    "L-Table Pick & Place (thin box)": {
        "id": "Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0",
        "mimic_id": "Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0",
        "dataset": "ffw_sg2_l_table_raw.hdf5",
        "robot": "FFW_SG2",
    },
    "Box Stack (thick box)": {
        "id": "Cyclo-Real-Box-Stack-FFW-SG2-v0",
        "mimic_id": "Cyclo-Real-Mimic-Box-Stack-FFW-SG2-v0",
        "dataset": "ffw_sg2_box_stack_raw.hdf5",
        "robot": "FFW_SG2",
    },
    "Single Box Far (rear table)": {
        "id": "Cyclo-Real-Single-Box-Far-FFW-SG2-v0",
        "mimic_id": "Cyclo-Real-Mimic-Single-Box-Far-FFW-SG2-v0",
        "dataset": "ffw_sg2_single_box_far_raw.hdf5",
        "robot": "FFW_SG2",
    },
    "Single Box Far (thick box)": {
        "id": "Cyclo-Real-Single-Box-Far-Thick-FFW-SG2-v0",
        "mimic_id": "Cyclo-Real-Mimic-Single-Box-Far-Thick-FFW-SG2-v0",
        "dataset": "ffw_sg2_single_box_far_thick_raw.hdf5",
        "robot": "FFW_SG2",
    },
    "L-Table Pick & Place (thin, hands)": {
        "id": "Cyclo-Real-Pick-Place-LTable-FFW-SH5-v0",
        "mimic_id": "Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SH5-v0",
        "dataset": "ffw_sh5_l_table_raw.hdf5",
        "robot": "FFW_SH5",
    },
    "Box Stack (thick, hands)": {
        "id": "Cyclo-Real-Box-Stack-FFW-SH5-v0",
        "mimic_id": "Cyclo-Real-Mimic-Box-Stack-FFW-SH5-v0",
        "dataset": "ffw_sh5_box_stack_raw.hdf5",
        "robot": "FFW_SH5",
    },
    "Single Box Far (rear, hands)": {
        "id": "Cyclo-Real-Single-Box-Far-FFW-SH5-v0",
        "mimic_id": "Cyclo-Real-Mimic-Single-Box-Far-FFW-SH5-v0",
        "dataset": "ffw_sh5_single_box_far_raw.hdf5",
        "robot": "FFW_SH5",
    },
    "Single Box Far (thick, hands)": {
        "id": "Cyclo-Real-Single-Box-Far-Thick-FFW-SH5-v0",
        "mimic_id": "Cyclo-Real-Mimic-Single-Box-Far-Thick-FFW-SH5-v0",
        "dataset": "ffw_sh5_single_box_far_thick_raw.hdf5",
        "robot": "FFW_SH5",
    },
}
TASK = os.environ.get("TASK", "Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0")
# Resolve the initial selected label from TASK (fall back to first entry).
_selected_task = next(
    (label for label, t in TASKS.items() if t["id"] == TASK),
    next(iter(TASKS)),
)
ROBOT_TYPE = os.environ.get("ROBOT_TYPE", "FFW_SG2")
NUM_DEMOS = os.environ.get("NUM_DEMOS", "0")
AUTO_SUCCESS = os.environ.get("AUTO_SUCCESS", "0") in ("1", "true", "True")
PIPELINE_STEPS = ("ik", "annotate", "generate", "joint", "lerobot")
PIPELINE_STEP_INPUT = {
    "ik": "raw",
    "annotate": "ik",
    "generate": "annotate",
    "joint": "generate",
    "lerobot": "joint",
}
PIPELINE_PROC_PATTERN = {
    "ik": "[a]ction_data_converter.py",
    "annotate": "[a]nnotate_demos.py",
    "generate": "[g]enerate_dataset.py",
    "joint": "[a]ction_data_converter.py",
    "lerobot": "[i]saaclab2lerobot.py",
}
PIPELINE_STEP_LABEL = {
    "ik": "IK convert",
    "annotate": "Annotate",
    "generate": "Datagen",
    "joint": "Joint convert",
    "lerobot": "LeRobot export",
}
GENERATION_NUM_TRIALS = os.environ.get("GENERATION_NUM_TRIALS", "500")
PIPELINE_NUM_ENVS = os.environ.get("PIPELINE_NUM_ENVS", "10")
CYCLO_C = "cyclo_lab"
VR_C = "robotis-applications"
AI_C = "ai_worker"
REPO_IN = "/workspace/cyclo_lab"
ISAAC_PY = f"{REPO_IN}/third_party/IsaacLab/_isaac_sim/python.sh"
ROS_SETUP = "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash"
ENV_ROS = f"export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && export RMW_IMPLEMENTATION={RMW}"

_lock = threading.Lock()
_logs: dict[str, list[str]] = {
    k: [] for k in ("cyclo", "vr", "ai", "recorder", "isaac", "pipeline", *(
        f"pipe_{s}" for s in PIPELINE_STEPS
    ))
}
_status: dict[str, str] = {k: "stopped" for k in _logs}
_launch_ts: dict[str, float] = {}
_active_teleop_robot: str | None = None
_auto_success: bool = AUTO_SUCCESS
STARTING_GRACE_S = 90.0
TELEOP_WARMUP_S = 6.0


def _log(key: str, msg: str) -> None:
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    with _lock:
        _logs[key].append(line)
        if len(_logs[key]) > 400:
            _logs[key] = _logs[key][-400:]


def _set_status(key: str, status: str) -> None:
    with _lock:
        _status[key] = status


def _current_task() -> dict[str, str]:
    with _lock:
        label = _selected_task
    return {"label": label, **TASKS[label]}


def _set_task(label: str) -> bool:
    global _selected_task
    if label not in TASKS:
        return False
    with _lock:
        prev_robot = TASKS.get(_selected_task, {}).get("robot")
        _selected_task = label
        new_robot = TASKS[label]["robot"]
    _log("cyclo", f"task selected: {label} ({TASKS[label]['id']})")
    if prev_robot and prev_robot != new_robot:
        _log(
            "cyclo",
            f"robot changed {prev_robot} -> {new_robot}; "
            "Launch Record will restart VR + controller for the new robot.",
        )
    return True


def _set_robot(robot: str) -> bool:
    if robot not in ROBOT_PROFILES:
        return False
    for label, task in TASKS.items():
        if task["robot"] == robot:
            return _set_task(label)
    return False


def _set_auto_success(enabled: bool) -> None:
    global _auto_success
    with _lock:
        _auto_success = enabled
    mode = "auto (task success)" if enabled else "manual (N / right trigger)"
    _log("cyclo", f"record save mode: {mode}")


def _run(cmd: list[str], cwd: Path | None = None) -> tuple[int, str]:
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=120)
        out = (r.stdout or "") + (r.stderr or "")
        return r.returncode, out.strip()
    except Exception as e:
        return 1, str(e)


def _docker_running(name: str) -> bool:
    code, out = _run(["docker", "inspect", "-f", "{{.State.Running}}", name])
    return code == 0 and out.strip() == "true"


def _start_container(script: Path) -> tuple[bool, str]:
    if not script.is_file():
        return False, f"missing {script}"
    code, out = _run(["bash", str(script), "start"], cwd=script.parent)
    return code == 0, out or ("ok" if code == 0 else "failed")


def _host_ip() -> str:
    code, out = _run(["hostname", "-I"])
    if code == 0 and out.strip():
        return out.strip().split()[0]
    return "127.0.0.1"


def _vuer_urls(host: str | None = None) -> dict[str, str]:
    """Vuer page + WebSocket URLs. Quest browser needs the combined ?ws= form."""
    host = host or _host_ip()
    base = f"https://{host}:8012"
    ws = f"wss://{host}:8012"
    return {
        "vuer_url": base,
        "vuer_ws": ws,
        "vuer_quest_url": f"{base}?ws={ws}",
        "vuer_quest_url_usb": "https://localhost:8012?ws=wss://localhost:8012",
    }


def _exec_detached(key: str, container: str, bash_cmd: str, log: str) -> tuple[bool, str]:
    # Run the command in the FOREGROUND of the detached exec (no trailing '&').
    # `docker exec -d` keeps the process alive as long as this bash stays in the
    # foreground; backgrounding it with nohup/& makes Docker reap the tree
    # immediately and kill the child.
    inner = f"exec > {log} 2>&1; {bash_cmd}"
    _log(key, f"start -> {log}")
    code, out = _run(["docker", "exec", "-d", container, "bash", "-lc", inner])
    if code == 0:
        _set_status(key, "starting")
        with _lock:
            _launch_ts[key] = time.time()
        return True, "started"
    _set_status(key, "failed")
    return False, out or "exec failed"


def _spawn(key: str, container: str, bash_cmd: str, *, log_suffix: str | None = None) -> tuple[bool, str]:
    suffix = log_suffix if log_suffix is not None else key
    return _exec_detached(key, container, bash_cmd, f"/tmp/sg2_ltable_{suffix}.log")


def _container_file_exists(path: str) -> bool:
    code, _ = _run(["docker", "exec", CYCLO_C, "test", "-f", path])
    return code == 0


def _pipeline_step_running(step: str) -> bool:
    return _proc_running(CYCLO_C, PIPELINE_PROC_PATTERN[step])


def _any_pipeline_running() -> str | None:
    for step in PIPELINE_STEPS:
        if _pipeline_step_running(step):
            return step
    return None


def launch_containers() -> dict:
    results = {}
    for name, script in [
        ("cyclo_lab", CYCLO / "docker" / "container.sh"),
        ("robotis_applications", VR_REPO / "docker" / "container.sh"),
        ("ai_worker", AI_REPO / "docker" / "container.sh"),
    ]:
        ok, msg = _start_container(script)
        results[name] = {"ok": ok, "msg": msg[-500:] if msg else ""}
        _log("cyclo", f"container {name}: {'ok' if ok else 'FAIL'}")
    return results


def _cleanup(container: str, patterns: list[str]) -> None:
    # Run pkill in a SEPARATE exec from the launch. If pkill and the launch
    # command share a shell, `pkill -f <name>` matches that shell's own command
    # line (which contains <name>) and kills it before the launch runs.
    # Bracket the first char so pkill can't match this cleanup shell either.
    parts = "; ".join(f'pkill -9 -f "[{p[0]}]{p[1:]}" 2>/dev/null' for p in patterns)
    _run(["docker", "exec", container, "bash", "-lc", f"{parts}; sleep 1; true"])


def _current_robot_profile() -> dict[str, str]:
    robot = _current_task().get("robot", "FFW_SG2")
    return ROBOT_PROFILES.get(robot, ROBOT_PROFILES["FFW_SG2"])


def _running_vr_model() -> str | None:
    """Return 'sg2' or 'sh5' if a vr launch process is running, else None."""
    code, out = _run(
        [
            "docker",
            "exec",
            VR_C,
            "bash",
            "-lc",
            "ps aux 2>/dev/null | grep -E '[v]r.launch|[v]r_publisher' | head -5",
        ]
    )
    if code != 0 or not out:
        return None
    if "model:=sh5" in out or "ffw_sh5" in out:
        return "sh5"
    if "model:=sg2" in out or "ffw_sg2" in out:
        return "sg2"
    return None


def _running_ai_hand() -> str | None:
    """Return 'true' or 'false' from the running motion-controller launch, else None."""
    code, out = _run(
        [
            "docker",
            "exec",
            AI_C,
            "bash",
            "-lc",
            "ps aux 2>/dev/null | grep -E '[a]i_worker_controller.launch|[v]r_controller_node' | head -5",
        ]
    )
    if code != 0 or not out:
        return None
    if "hand:=true" in out:
        return "true"
    if "hand:=false" in out:
        return "false"
    return None


def _teleop_stack_matches(profile: dict[str, str]) -> bool:
    vr = _running_vr_model()
    hand = _running_ai_hand()
    if vr is None or hand is None:
        return False
    return vr == profile["vr_model"] and hand == profile["hand"]


def ensure_teleop_stack() -> dict:
    """Kill and relaunch VR + motion controller for the currently selected robot."""
    global _active_teleop_robot
    profile = _current_robot_profile()
    task = _current_task()
    _log(
        "cyclo",
        f"teleop stack -> {profile['robot_type']} "
        f"(vr model={profile['vr_model']}, hand={profile['hand']}) for task: {task['label']}",
    )
    results: dict = {"profile": profile, "task": task["label"]}
    ok_vr, msg_vr = launch_vr()
    results["vr"] = {"ok": ok_vr, "msg": msg_vr}
    time.sleep(2)
    ok_ai, msg_ai = launch_ai_stack()
    results["ai"] = {"ok": ok_ai, "msg": msg_ai}
    if ok_vr and ok_ai:
        with _lock:
            _active_teleop_robot = profile["robot_type"]
        time.sleep(TELEOP_WARMUP_S)
    else:
        with _lock:
            _active_teleop_robot = None
    return results


def launch_ai_stack() -> tuple[bool, str]:
    profile = _current_robot_profile()
    _log("ai", f"motion controller hand:={profile['hand']} ({profile['robot_type']})")
    _cleanup(AI_C, ["ros2_control_node", "robot_state_publisher",
                    "ai_worker_controller", "vr_controller_node", "retargeting"])
    cmd = (
        f"{ENV_ROS} && {ROS_SETUP} && "
        f"U={profile['urdf']}; "
        "ros2 run robot_state_publisher robot_state_publisher "
        '--ros-args -p robot_description:="$(cat $U)" & '
        "sleep 4; "
        "ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py "
        f"controller_type:=vr hand:={profile['hand']}"
    )
    return _spawn("ai", AI_C, cmd)


def launch_vr() -> tuple[bool, str]:
    profile = _current_robot_profile()
    extra = profile.get("vr_extra_args", "")
    if profile["vr_model"] == "sh5" and not LIFT_ENABLED:
        # Allow disabling lift via SH5_ENABLE_LIFT=0 (e.g. for stable manipulation).
        extra = extra.replace("enable_lift_publishing:=true", "enable_lift_publishing:=false")
    _log(
        "vr",
        f"VR launch model:={profile['vr_model']} ({profile['robot_type']})"
        + (f" [{extra}]" if extra else ""),
    )
    _cleanup(VR_C, ["vr_publisher", "vr.launch"])
    vr_image = "true" if VR_IMAGE_ENABLED else "false"
    cmd = (
        f"{ENV_ROS} && {ROS_SETUP} && "
        f"ros2 launch robotis_vuer vr.launch.py model:={profile['vr_model']} "
        f"enable_vr_image:={vr_image} {extra}".strip()
    )
    return _spawn("vr", VR_C, cmd)


def launch_recorder(*, ensure_teleop: bool = True) -> tuple[bool, str]:
    task = _current_task()
    profile = _current_robot_profile()
    if ensure_teleop:
        teleop = ensure_teleop_stack()
        if not teleop.get("vr", {}).get("ok") or not teleop.get("ai", {}).get("ok"):
            _log("recorder", "aborted: teleop stack failed to start for selected robot")
            return False, "teleop stack failed; see vr/ai logs"
    ds = f"{REPO_IN}/datasets/{task['dataset']}"
    auto_flag = " --auto_success" if _auto_success else ""
    _log("recorder", f"task: {task['label']} -> {task['id']} ({profile['robot_type']})")
    _log("recorder", f"save mode: {'auto success' if _auto_success else 'manual'}")
    cmd = (
        f"cd {REPO_IN} && export DISPLAY=:1 && {ENV_ROS} && "
        f"{ISAAC_PY} scripts/sim2real/imitation_learning/recorder/record_demos.py "
        f"--task={task['id']} --robot_type {profile['robot_type']} "
        f"--dataset_file {ds} --num_demos {NUM_DEMOS}{auto_flag} --enable_cameras"
    )
    return _spawn("recorder", CYCLO_C, cmd)


def kill_isaac() -> tuple[bool, str]:
    # The recorder runs as: python.sh -> kit/python3 (record_demos.py) -> carb threads.
    # Match record_demos.py (hits both the python.sh wrapper and the kit python)
    # plus any leftover Isaac kit process. Bracket the first char so pkill can't
    # match the shell running this command.
    cmd = (
        'pkill -9 -f "[r]ecord_demos.py" 2>/dev/null; '
        'pkill -9 -f "[_]isaac_sim/kit" 2>/dev/null; '
        'pkill -9 -f "[i]saac-sim" 2>/dev/null; '
        "sleep 1; "
        'pgrep -f "[r]ecord_demos.py" >/dev/null && echo "still-running" || echo "killed"'
    )
    code, out = _run(["docker", "exec", CYCLO_C, "bash", "-lc", cmd])
    killed = "killed" in (out or "")
    _set_status("isaac", "stopped" if killed else "running")
    _set_status("recorder", "stopped" if killed else "running")
    _log("isaac", f"kill -> {out.strip() if out else '(no output)'}")
    return killed, out or "no output"


def launch_all() -> dict:
    out: dict = {"containers": launch_containers()}
    time.sleep(3)
    out["teleop"] = ensure_teleop_stack()
    out.update(_vuer_urls())
    profile = _current_robot_profile()
    out["note"] = (
        f"VR + controller running for {profile['robot_type']} "
        f"(model={profile['vr_model']}, hand={profile['hand']}). "
        "Use Launch Record to start Isaac with the matching stack."
    )
    return out


def launch_record() -> dict:
    """Ensure containers, correct VR/AI for selected robot, then start Isaac recorder."""
    out: dict = {"containers": launch_containers()}
    time.sleep(3)
    out["teleop"] = ensure_teleop_stack()
    ok, msg = launch_recorder(ensure_teleop=False)
    out["recorder"] = {"ok": ok, "msg": msg}
    out.update(_vuer_urls())
    return out


def _pipeline_paths(task: dict[str, str]) -> dict[str, str]:
    """Derive mimic-pipeline HDF5 paths from the task raw dataset name."""
    raw_name = task["dataset"]
    if raw_name.endswith("_raw.hdf5"):
        base = raw_name[: -len("_raw.hdf5")]
    else:
        base = raw_name.removesuffix(".hdf5")
    ds = f"{REPO_IN}/datasets"
    return {
        "raw": f"{ds}/{raw_name}",
        "ik": f"{ds}/{base}_ik.hdf5",
        "annotate": f"{ds}/{base}_annotate.hdf5",
        "generate": f"{ds}/{base}_generate.hdf5",
        "joint": f"{ds}/{base}_joint.hdf5",
    }


def _pipeline_cmd(step: str) -> tuple[str, str]:
    """Return (log label, bash command) for a single pipeline step inside cyclo_lab container."""
    task = _current_task()
    profile = _current_robot_profile()
    paths = _pipeline_paths(task)
    robot = profile["robot_type"]
    mimic_id = task["mimic_id"]
    record_id = task["id"]
    py = ISAAC_PY
    label = PIPELINE_STEP_LABEL[step]
    if step == "ik":
        cmd = (
            f"cd {REPO_IN} && {py} scripts/sim2real/imitation_learning/mimic/action_data_converter.py "
            f"--robot_type {robot} --input_file {paths['raw']} --output_file {paths['ik']} --action_type ik"
        )
    elif step == "annotate":
        cmd = (
            f"cd {REPO_IN} && {py} scripts/sim2real/imitation_learning/mimic/annotate_demos.py "
            f"--task {mimic_id} --auto --input_file {paths['ik']} --output_file {paths['annotate']} "
            "--enable_cameras --headless"
        )
    elif step == "generate":
        cmd = (
            f"cd {REPO_IN} && {py} scripts/sim2real/imitation_learning/mimic/generate_dataset.py "
            f"--device cuda --num_envs {PIPELINE_NUM_ENVS} --task {mimic_id} "
            f"--generation_num_trials {GENERATION_NUM_TRIALS} --input_file {paths['annotate']} "
            f"--output_file {paths['generate']} --enable_cameras --headless"
        )
    elif step == "joint":
        cmd = (
            f"cd {REPO_IN} && {py} scripts/sim2real/imitation_learning/mimic/action_data_converter.py "
            f"--robot_type {robot} --input_file {paths['generate']} --output_file {paths['joint']} "
            "--action_type joint"
        )
    else:  # lerobot
        cmd = (
            f"cd {REPO_IN} && lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py "
            f"--task={record_id} --robot_type {robot} --dataset_file {paths['joint']}"
        )
    return label, cmd


def _wait_pipeline_step(step: str, timeout_s: float = 14400.0) -> bool:
    """Wait until a detached pipeline step process exits."""
    key = f"pipe_{step}"
    deadline = time.time() + timeout_s
    # Give the process a moment to show up in pgrep.
    time.sleep(2.0)
    while time.time() < deadline:
        if not _pipeline_step_running(step):
            with _lock:
                cur = _status.get(key)
            if cur == "starting":
                _set_status(key, "stopped")
            return True
        time.sleep(3.0)
    _log("pipeline", f"{step}: timed out after {timeout_s:.0f}s")
    return False


def launch_pipeline_step(step: str) -> tuple[bool, str]:
    if step not in PIPELINE_STEPS:
        return False, f"unknown step: {step}"
    if not _docker_running(CYCLO_C):
        return False, f"{CYCLO_C} container is not running"
    busy = _any_pipeline_running()
    if busy is not None:
        return False, f"step '{busy}' is still running; wait or Kill pipeline"
    task = _current_task()
    paths = _pipeline_paths(task)
    input_key = PIPELINE_STEP_INPUT[step]
    input_path = paths[input_key]
    if not _container_file_exists(input_path):
        return False, f"missing input for {step}: {input_path}"
    label, cmd = _pipeline_cmd(step)
    key = f"pipe_{step}"
    _log("pipeline", f"start {label} for {task['label']}")
    _log(key, f"input: {input_path}")
    return _spawn(key, CYCLO_C, cmd, log_suffix=f"pipe_{step}")


def launch_pipeline_full() -> None:
    task = _current_task()
    _log("pipeline", f"full mimic pipeline for {task['label']} (sequential)")
    for step in PIPELINE_STEPS:
        ok, msg = launch_pipeline_step(step)
        if not ok:
            _log("pipeline", f"stopped at {step}: {msg}")
            return
        _log("pipeline", f"waiting for {step} to finish...")
        if not _wait_pipeline_step(step):
            _log("pipeline", f"failed while waiting on {step}")
            return
        _log("pipeline", f"finished {step}")
    _log("pipeline", "full pipeline complete")


def kill_pipeline() -> tuple[bool, str]:
    cmd = (
        'pkill -9 -f "[a]ction_data_converter.py" 2>/dev/null; '
        'pkill -9 -f "[a]nnotate_demos.py" 2>/dev/null; '
        'pkill -9 -f "[g]enerate_dataset.py" 2>/dev/null; '
        'pkill -9 -f "[i]saaclab2lerobot.py" 2>/dev/null; '
        "sleep 1; "
        'pgrep -f "[g]enerate_dataset.py|[a]nnotate_demos.py|[a]ction_data_converter.py|[i]saaclab2lerobot.py" >/dev/null && echo "still-running" || echo "killed"'
    )
    code, out = _run(["docker", "exec", CYCLO_C, "bash", "-lc", cmd])
    killed = "killed" in (out or "")
    for step in PIPELINE_STEPS:
        _set_status(f"pipe_{step}", "stopped")
    _log("pipeline", f"kill -> {out.strip() if out else '(no output)'}")
    return killed, out or "no output"


def _proc_running(container: str, pattern: str) -> bool:
    code, out = _run(
        ["docker", "exec", container, "bash", "-lc", f"pgrep -f '{pattern}' >/dev/null && echo yes || echo no"]
    )
    return code == 0 and out.strip() == "yes"


def _reconcile(key: str, container_up: bool, container: str, pattern: str) -> None:
    """Make status reflect reality. 'starting' is preserved until the process
    actually appears or the container goes away, so the UI shows progress."""
    if not container_up:
        _set_status(key, "stopped")
        return
    if _proc_running(container, pattern):
        _set_status(key, "running")
        return
    with _lock:
        cur = _status.get(key)
        ts = _launch_ts.get(key, 0.0)
    if cur == "starting" and (time.time() - ts) < STARTING_GRACE_S:
        return
    _set_status(key, "stopped")


def _read_teleop_grip() -> dict:
    try:
        return json.loads(TELEOP_GRIP_STATUS.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"state": "unknown", "held_s": 0.0, "auto_l_in_s": None, "auto_l_enabled": False}


def snapshot() -> dict:
    # Bracket the first char of each pattern so `pgrep -f` does not match the
    # shell that is running pgrep (its own command line contains the pattern).
    containers = {n: _docker_running(n) for n in (CYCLO_C, VR_C, AI_C)}
    _reconcile("vr", containers.get(VR_C, False), VR_C, "[v]r_publisher")
    _reconcile("ai", containers.get(AI_C, False), AI_C, "[v]r_controller_node")
    _reconcile("recorder", containers.get(CYCLO_C, False), CYCLO_C, "[r]ecord_demos.py")
    _reconcile("isaac", containers.get(CYCLO_C, False), CYCLO_C, "[i]saac-sim|[k]it/kit")
    for step in PIPELINE_STEPS:
        _reconcile(
            f"pipe_{step}",
            containers.get(CYCLO_C, False),
            CYCLO_C,
            PIPELINE_PROC_PATTERN[step],
        )
    with _lock:
        st = dict(_status)
        logs_tail = {k: v[-30:] for k, v in _logs.items()}
        auto_success = _auto_success
    task = _current_task()
    profile = _current_robot_profile()
    paths = _pipeline_paths(task)
    inputs_ready = {
        step: _container_file_exists(paths[PIPELINE_STEP_INPUT[step]]) if containers.get(CYCLO_C) else False
        for step in PIPELINE_STEPS
    }
    step_status = {step: st.get(f"pipe_{step}", "stopped") for step in PIPELINE_STEPS}
    vr_running = _running_vr_model()
    hand_running = _running_ai_hand()
    teleop_ok = _teleop_stack_matches(profile)
    return {
        "containers": containers,
        "status": st,
        **_vuer_urls(),
        "task": task["id"],
        "task_label": task["label"],
        "robot": task.get("robot", "FFW_SG2"),
        "robot_label": "Gripper (SG2)" if task.get("robot") == "FFW_SG2" else "Hands (SH5)",
        "vr_model": profile["vr_model"],
        "hand": profile["hand"],
        "teleop_matches_task": teleop_ok,
        "running_vr_model": vr_running,
        "running_ai_hand": hand_running,
        "grip": _read_teleop_grip(),
        "auto_success": auto_success,
        "tasks": list(TASKS.keys()),
        "pipeline": {
            "paths": paths,
            "mimic_id": task["mimic_id"],
            "generation_num_trials": GENERATION_NUM_TRIALS,
            "num_envs": PIPELINE_NUM_ENVS,
            "steps": step_status,
            "inputs_ready": inputs_ready,
        },
        "tasks_by_robot": {
            robot: [label for label, t in TASKS.items() if t["robot"] == robot]
            for robot in ROBOT_PROFILES
        },
        "logs": logs_tail,
    }


HTML = r"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SG2 L-Table Launcher</title>
<style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;background:#0f1117;color:#e8eaed}
header{padding:1rem 1.5rem;background:#1a1d27;border-bottom:1px solid #2a2f3d}
h1{margin:0;font-size:1.25rem}main{padding:1.5rem;max-width:1100px;margin:0 auto}
.row{display:flex;flex-wrap:wrap;gap:.75rem;margin-bottom:1rem}
button{padding:.6rem 1rem;border:0;border-radius:8px;cursor:pointer;font-weight:600}
.primary{background:#3b82f6;color:#fff}.danger{background:#ef4444;color:#fff}
.secondary{background:#374151;color:#fff}.ok{color:#4ade80}.bad{color:#f87171}
.card{background:#1a1d27;border:1px solid #2a2f3d;border-radius:10px;padding:1rem;margin-bottom:1rem}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:.5rem}
.tag{padding:.35rem .6rem;border-radius:6px;background:#252a36;font-size:.85rem}
select{background:#0a0c10;color:#e8eaed;border:1px solid #2a2f3d;border-radius:6px;padding:.3rem .5rem;margin-left:.5rem;font-size:.85rem}
pre{background:#0a0c10;padding:.75rem;border-radius:8px;overflow:auto;max-height:220px;font-size:.75rem}
a{color:#60a5fa}
</style></head><body>
<header><h1>FFW VR + Recorder Dashboard</h1></header>
<main>
<div class="row">
<label class="tag">Robot:
<select id="robot" onchange="setRobot()">
<option value="FFW_SG2">Gripper (SG2)</option>
<option value="FFW_SH5">Hands (SH5)</option>
</select>
</label>
<label class="tag">Task:
<select id="task" onchange="setTask()"></select>
</label>
<label class="tag">Save episode:
<select id="auto_success" onchange="setAutoSuccess()">
<option value="manual">Manual (N key)</option>
<option value="auto">Auto on task success</option>
</select>
</label>
</div>
<div class="row">
<button class="primary" onclick="act('launch_all')">Launch VR + Controller</button>
<button class="primary" onclick="act('launch_record')">Launch Record</button>
<button class="secondary" onclick="act('launch_recorder')">Isaac Only</button>
<button class="danger" onclick="act('kill_isaac')">Kill Isaac</button>
<button class="secondary" onclick="refresh()">Refresh</button>
</div>
<div class="card"><h3>Mimic pipeline</h3>
<p class="tag">Run steps <b>one at a time</b> in order (wait for each to finish). Logs: <code>/tmp/sg2_ltable_pipe_&lt;step&gt;.log</code> in cyclo_lab container.</p>
<div class="row" id="pipeline_buttons"></div>
<div class="row">
<button class="primary" onclick="pipe('full')">Run all steps sequentially</button>
<button class="danger" onclick="act('kill_pipeline')">Kill pipeline</button>
</div>
<pre id="pipeline_paths" style="font-size:.7rem;margin-top:.5rem"></pre>
</div>
<div class="card"><div id="meta"></div><div class="grid" id="status"></div></div>
<div class="card"><h3>Logs</h3><pre id="logs"></pre></div>
<p><b>Launch Record</b> restarts VR + motion controller for the selected robot, then starts Isaac.
Gripper tasks use <code>model:=sg2 hand:=false</code>; hand tasks use <code>model:=sh5 hand:=true</code>.
VR: accept cert at Vuer URL. SG2: squeeze both grips. SH5: hand gesture to toggle publishing; <code>I</code>/<code>O</code> lift up/down in Isaac. Recording: <code>B</code> start, <code>N</code> save (manual default), <code>R</code> reset, <code>L</code> L-motion.
B=record, L=face target table (or auto after 2s gripped), N=save, R=reset.</p>
</main>
<script>
async function act(a){await fetch('/api/'+a,{method:'POST'});refresh()}
async function pipe(step){
 const r=await fetch('/api/pipeline/'+step,{method:'POST'});
 const j=await r.json();
 if(!j.ok&&j.msg){alert(j.msg)}
 refresh();
}
const PIPE_STEPS=[
 {id:'ik',n:'1. IK convert'},
 {id:'annotate',n:'2. Annotate'},
 {id:'generate',n:'3. Datagen'},
 {id:'joint',n:'4. Joint convert'},
 {id:'lerobot',n:'5. LeRobot export'},
];
function renderPipelineButtons(d){
 const el=document.getElementById('pipeline_buttons');
 if(!el)return;
 const steps=(d.pipeline&&d.pipeline.steps)||{};
 const ready=(d.pipeline&&d.pipeline.inputs_ready)||{};
 const anyRun=Object.values(steps).some(s=>s==='running'||s==='starting');
 el.innerHTML=PIPE_STEPS.map(s=>{
  const st=steps[s.id]||'stopped';
  const cls=st==='running'?'ok':(st==='starting'?'':'');
  const miss=ready[s.id]===false?' (input missing)':'';
  const dis=anyRun&&(st!=='running'&&st!=='starting')?' disabled':'';
  return '<button class="secondary"'+dis+' onclick="pipe(\''+s.id+'\')">'+s.n+
   ' <span class="tag '+cls+'">'+st+miss+'</span></button>';
 }).join('');
}
async function setTask(){
 const label=document.getElementById('task').value;
 await fetch('/api/set_task',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({task:label})});
 refresh();
}
async function setRobot(){
 const robot=document.getElementById('robot').value;
 await fetch('/api/set_robot',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({robot})});
 refresh();
}
async function setAutoSuccess(){
 const mode=document.getElementById('auto_success').value;
 await fetch('/api/set_auto_success',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({auto_success:mode==='auto'})});
 refresh();
}
function fillTaskOptions(d){
 const sel=document.getElementById('task');
 const robot=document.getElementById('robot').value;
 const tasks=(d.tasks_by_robot&&d.tasks_by_robot[robot])||d.tasks;
 sel.innerHTML='';
 tasks.forEach(t=>{const o=document.createElement('option');o.value=t;o.textContent=t;sel.appendChild(o);});
 if(tasks.includes(d.task_label)) sel.value=d.task_label;
 else if(tasks.length) sel.value=tasks[0];
}
async function refresh(){
 const d=await(await fetch('/api/status')).json();
 const robotSel=document.getElementById('robot');
 if(document.activeElement!==robotSel) robotSel.value=d.robot;
 fillTaskOptions(d);
 const sel=document.getElementById('task');
 if(document.activeElement!==sel) sel.value=d.task_label;
 const autoSel=document.getElementById('auto_success');
 if(document.activeElement!==autoSel) autoSel.value=d.auto_success?'auto':'manual';
 const teleopCls=d.teleop_matches_task?'ok':'bad';
 const teleopTxt=d.teleop_matches_task?'matches task':'will restart on Launch Record';
 let h='<p>Robot: <b>'+d.robot_label+'</b> <span class="tag">'+d.robot+'</span><br>';
 h+='Task: <b>'+d.task_label+'</b> <span class="tag">'+d.task+'</span><br>';
 h+='Expected teleop: <span class="tag">vr model='+d.vr_model+', hand='+d.hand+'</span><br>';
 h+='Running teleop: <span class="tag">vr='+(d.running_vr_model||'none')+', hand='+(d.running_ai_hand||'none')+'</span> ';
 h+='<span class="'+teleopCls+'">('+teleopTxt+')</span><br>';
 const g=d.grip||{};
 const gripCls=g.state==='gripped'?'ok':'';
 let gripTxt=g.state||'unknown';
 if(g.state==='gripped'&&g.auto_l_enabled&&g.auto_l_in_s!=null){
  gripTxt+=' (auto-L in '+g.auto_l_in_s+'s)';
 }
 h+='Grip: <span class="tag '+gripCls+'">'+gripTxt+'</span><br>';
 h+='Save mode: <span class="tag">'+(d.auto_success?'auto success':'manual (N)')+'</span><br>';
 h+='Quest (WiFi): <code style="word-break:break-all">'+d.vuer_quest_url+'</code><br>';
 h+='Quest (USB/ADB): <code style="word-break:break-all">'+d.vuer_quest_url_usb+'</code><br>';
 h+='Vuer page: <a href="'+d.vuer_url+'" target="_blank">'+d.vuer_url+'</a> · WS: '+d.vuer_ws+'</p>';
 document.getElementById('meta').innerHTML=h;
 let s='';
 for(const[k,v]of Object.entries(d.containers))s+='<div class="tag">'+k+': <span class="'+(v?'ok':'bad')+'">'+(v?'up':'down')+'</span></div>';
 for(const[k,v]of Object.entries(d.status))s+='<div class="tag">'+k+': '+v+'</div>';
 document.getElementById('status').innerHTML=s;
 let lg='';for(const[k,lines]of Object.entries(d.logs)){lg+='=== '+k+' ===\n'+lines.join('\n')+'\n\n'}
 document.getElementById('logs').textContent=lg;
 if(d.pipeline&&d.pipeline.paths){
  renderPipelineButtons(d);
  const p=d.pipeline.paths;
  document.getElementById('pipeline_paths').textContent=
   'raw: '+p.raw+'\nik: '+p.ik+'\nannotate: '+p.annotate+'\ngenerate: '+p.generate+'\njoint: '+p.joint+
   '\nmimic: '+d.pipeline.mimic_id+' | trials: '+d.pipeline.generation_num_trials+' | envs: '+d.pipeline.num_envs;
 }
}
refresh();setInterval(refresh,3000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        pass

    def _json(self, code: int, data: dict) -> None:
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            body = HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/status":
            self._json(200, snapshot())
            return
        self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/launch_all":
            threading.Thread(target=launch_all, daemon=True).start()
            self._json(200, {"ok": True, "msg": "launch started"})
            return
        if path == "/api/launch_record":
            threading.Thread(target=launch_record, daemon=True).start()
            self._json(200, {"ok": True, "msg": "launch record started"})
            return
        if path == "/api/launch_recorder":
            ok, msg = launch_recorder()
            self._json(200, {"ok": ok, "msg": msg})
            return
        if path == "/api/kill_isaac":
            ok, msg = kill_isaac()
            self._json(200, {"ok": ok, "msg": msg})
            return
        if path.startswith("/api/pipeline/"):
            step = path.rsplit("/", 1)[-1]
            if step == "full":
                threading.Thread(target=launch_pipeline_full, daemon=True).start()
                self._json(200, {"ok": True, "msg": "full pipeline started"})
                return
            if step in PIPELINE_STEPS:
                ok, msg = launch_pipeline_step(step)
                self._json(200, {"ok": ok, "msg": msg})
                return
            self._json(400, {"error": f"unknown pipeline step: {step}"})
            return
        if path == "/api/kill_pipeline":
            ok, msg = kill_pipeline()
            self._json(200, {"ok": ok, "msg": msg})
            return
        if path == "/api/set_task":
            length = int(self.headers.get("Content-Length", "0") or "0")
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                body = {}
            ok = _set_task(body.get("task", ""))
            self._json(200 if ok else 400, {"ok": ok, "task": _current_task()})
            return
        if path == "/api/set_robot":
            length = int(self.headers.get("Content-Length", "0") or "0")
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                body = {}
            ok = _set_robot(body.get("robot", ""))
            self._json(200 if ok else 400, {"ok": ok, "task": _current_task()})
            return
        if path == "/api/set_auto_success":
            length = int(self.headers.get("Content-Length", "0") or "0")
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                body = {}
            _set_auto_success(bool(body.get("auto_success", False)))
            self._json(200, {"ok": True, "auto_success": _auto_success})
            return
        self._json(404, {"error": "not found"})


def main() -> None:
    urls = _vuer_urls()
    print(f"Dashboard http://0.0.0.0:{PORT}")
    print(f"Quest Vuer (WiFi):  {urls['vuer_quest_url']}")
    print(f"Quest Vuer (USB):   {urls['vuer_quest_url_usb']}")
    HTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
