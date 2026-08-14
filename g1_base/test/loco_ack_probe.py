#!/usr/bin/env python3
"""
Probe Unitree G1 loco RPC ACK behavior with a single standalone client.

Recommended use on PC2 after stopping g1_base_manager / g1_control_server:

    python3 test/loco_ack_probe.py --net-if enP8p1s0 --send-zero-velocity

The script always performs read-only GET probes first. It only sends
SetVelocity(0, 0, 0) when --send-zero-velocity is provided.
"""

import argparse
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional


DEFAULT_SDK_PATHS = (
    "/home/unitree/unitree_sdk2_python",
    "/home/lemon/Super-LIO/thirdparty/unitree_sdk2_python",
)

CONFLICT_KEYWORDS = (
    "g1_base.unitree_sdk_bridge",
    "g1_base.g1_base_manager",
    "g1_control_server",
    "test_pc1_latency.py",
    "g1_loco_client_example",
)


@dataclass
class ProbeResult:
    label: str
    timeout: float
    elapsed_ms: float
    result: Any = None
    error: Optional[str] = None

    @property
    def code(self) -> Optional[int]:
        if self.error is not None:
            return None
        if isinstance(self.result, int):
            return self.result
        if isinstance(self.result, tuple) and self.result:
            first = self.result[0]
            return int(first) if isinstance(first, int) else None
        return None


def parse_timeouts(raw: str) -> list[float]:
    values = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    if not values:
        raise argparse.ArgumentTypeError("at least one timeout is required")
    return values


def add_sdk_path(explicit_path: Optional[str]) -> None:
    candidates: Iterable[str]
    if explicit_path:
        candidates = (explicit_path,)
    else:
        env_path = os.environ.get("UNITREE_SDK2_PYTHON", "").strip()
        candidates = (env_path, *DEFAULT_SDK_PATHS) if env_path else DEFAULT_SDK_PATHS

    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser()
        if path.is_dir():
            sys.path.insert(0, str(path))
            return


def find_conflicting_processes() -> list[str]:
    try:
        output = subprocess.check_output(
            ["ps", "-eo", "pid=,ppid=,args="],
            text=True,
        )
    except Exception:
        return []

    current_pid = os.getpid()
    conflicts = []
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split(None, 2)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid == current_pid:
            continue
        if any(keyword in stripped for keyword in CONFLICT_KEYWORDS):
            conflicts.append(stripped)
    return conflicts


def run_probe(
    label: str,
    timeout: float,
    set_timeout: Callable[[float], None],
    fn: Callable[[], Any],
) -> ProbeResult:
    set_timeout(timeout)
    start = time.perf_counter()
    try:
        result = fn()
        error = None
    except Exception as exc:  # keep probe going across failures
        result = None
        error = repr(exc)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return ProbeResult(label, timeout, elapsed_ms, result, error)


def print_result(item: ProbeResult) -> None:
    status = f"code={item.code}" if item.code is not None else "code=?"
    if item.error:
        status = f"error={item.error}"
    print(
        f"{item.label:<24} timeout={item.timeout:>4.1f}s "
        f"elapsed={item.elapsed_ms:>8.1f}ms {status} result={item.result!r}",
        flush=True,
    )


def summarize(results: list[ProbeResult]) -> None:
    get_results = [r for r in results if r.label.startswith("get_")]
    velocity_results = [
        r for r in results
        if r.label in ("set_velocity_zero", "move_zero")
    ]

    get_ok = any(r.code == 0 for r in get_results)
    velocity_ok = any(r.code == 0 for r in velocity_results)
    velocity_timeout = velocity_results and all(r.code == 3104 for r in velocity_results)

    print("\n=== interpretation ===")
    if get_ok and velocity_ok:
        print("GET works and zero velocity gets ACK at least once.")
    elif get_ok and velocity_timeout:
        print("GET works, but zero velocity timed out for every tested timeout.")
        print("This points to loco/sport accepting read-only RPC but not ACKing velocity.")
    elif get_ok and velocity_results:
        print("GET works, but zero velocity results are mixed; inspect each timeout above.")
    elif not get_ok:
        print("GET did not succeed; check DDS interface/domain or robot sport service.")
    else:
        print("Only read-only probes were run. Add --send-zero-velocity for ACK testing.")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Probe G1 loco service ACK timing")
    parser.add_argument("--net-if", default="enP8p1s0", help="DDS network interface")
    parser.add_argument("--domain", type=int, default=0, help="Unitree DDS domain")
    parser.add_argument(
        "--sdk-path",
        default=None,
        help="Path containing unitree_sdk2py; auto-detected when omitted",
    )
    parser.add_argument(
        "--timeouts",
        type=parse_timeouts,
        default=parse_timeouts("0.2,0.5,1.0"),
        help="Comma-separated RPC timeouts for velocity probes",
    )
    parser.add_argument(
        "--get-timeout",
        type=float,
        default=0.2,
        help="Timeout used for read-only GET probes",
    )
    parser.add_argument(
        "--velocity-duration",
        type=float,
        default=1.0,
        help="Duration field for SetVelocity(0,0,0,duration)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.3,
        help="Sleep between probes so late responses can drain",
    )
    parser.add_argument(
        "--send-zero-velocity",
        action="store_true",
        help="Actually send SetVelocity(0,0,0) probes",
    )
    parser.add_argument(
        "--also-move-zero",
        action="store_true",
        help="Also call Move(0,0,0) after SetVelocity probes",
    )
    parser.add_argument(
        "--allow-concurrent",
        action="store_true",
        help="Run even if g1_base SDK processes appear to be active",
    )
    args = parser.parse_args(argv)

    conflicts = find_conflicting_processes()
    if conflicts and not args.allow_concurrent:
        print("Refusing to probe while possible g1_base/SDK clients are active:")
        for line in conflicts:
            print(f"  {line}")
        print("Stop g1_base first, or pass --allow-concurrent if this is intentional.")
        return 3

    add_sdk_path(args.sdk_path)

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        import unitree_sdk2py.g1.loco.g1_loco_api as api
    except Exception as exc:
        print(f"Failed to import unitree_sdk2py: {exc}", file=sys.stderr)
        return 2

    print("=== loco ACK probe ===")
    print(f"net_if={args.net_if} domain={args.domain}")
    print(f"timeouts={','.join(str(x) for x in args.timeouts)}")
    print(f"send_zero_velocity={args.send_zero_velocity}")

    ChannelFactoryInitialize(args.domain, args.net_if)
    client = LocoClient()
    client.SetTimeout(args.get_timeout)
    client.Init()

    results: list[ProbeResult] = []

    readonly_probes = (
        ("get_server_api_version", client.GetServerApiVersion),
        ("get_fsm_id", lambda: client._Call(api.ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")),
        ("get_fsm_mode", lambda: client._Call(api.ROBOT_API_ID_LOCO_GET_FSM_MODE, "{}")),
        (
            "get_balance_mode",
            lambda: client._Call(api.ROBOT_API_ID_LOCO_GET_BALANCE_MODE, "{}"),
        ),
    )
    print("\n=== read-only GET probes ===")
    for label, fn in readonly_probes:
        item = run_probe(label, args.get_timeout, client.SetTimeout, fn)
        results.append(item)
        print_result(item)
        time.sleep(args.settle)

    if args.send_zero_velocity:
        print("\n=== zero velocity ACK probes ===")
        for timeout in args.timeouts:
            item = run_probe(
                "set_velocity_zero",
                timeout,
                client.SetTimeout,
                lambda: client.SetVelocity(
                    0.0,
                    0.0,
                    0.0,
                    duration=args.velocity_duration,
                ),
            )
            results.append(item)
            print_result(item)
            time.sleep(args.settle)

        if args.also_move_zero:
            for timeout in args.timeouts:
                item = run_probe(
                    "move_zero",
                    timeout,
                    client.SetTimeout,
                    lambda: client.Move(0.0, 0.0, 0.0),
                )
                results.append(item)
                print_result(item)
                time.sleep(args.settle)
    else:
        print("\nSkipped zero velocity probes. Add --send-zero-velocity to run them.")

    summarize(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
