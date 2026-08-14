"""Command-line interface for g1_teach_v2.

This module only parses arguments and dispatches subcommands. Recording,
playback, snapshots, and scripts stay in dedicated modules so they remain easy
to reuse from SSH sessions and future adapters.
"""

import argparse

from .audio_io import DEFAULT_AUDIO_VOLUME
from .hand_adapters import HAND_BACKENDS
from .iface_utils import AUTO_IFACE, IFACE_ENV_VAR, print_network_interfaces
from .joints import ARM_GROUPS


DEFAULT_IFACE = AUTO_IFACE


def build_parser():
    """Build the top-level argparse tree used by `python -m g1_teach_v2`."""
    parser = argparse.ArgumentParser(prog="python -m g1_teach_v2")
    subparsers = parser.add_subparsers(dest="command", required=True)
    iface_help = (
        "Robot network interface. Default: auto-detect. "
        f"Use --iface <name> to override or set {IFACE_ENV_VAR}."
    )

    iface_parser = subparsers.add_parser(
        "list-ifaces",
        help="List detected network interfaces and the recommended one",
    )
    iface_parser.set_defaults(func=handle_list_ifaces)

    record_parser = subparsers.add_parser("record-motion", help="Record a motion trajectory")
    record_parser.add_argument("--iface", default=DEFAULT_IFACE, help=iface_help)
    record_parser.add_argument(
        "--mode",
        required=True,
        choices=["raw", "teach_upper", "right_hold_left", "lock_forearm"],
        help="Record mode",
    )
    record_parser.add_argument("--out", required=True, help="Output motion jsonl path")
    record_parser.add_argument(
        "--group",
        default="upper",
        choices=sorted(ARM_GROUPS.keys()),
        help="Joint group for raw recording; other modes currently use upper only",
    )
    record_parser.add_argument(
        "--auto-hold",
        action="store_true",
        help="Experimental teach mode: freeze the current pose after the arm settles, then return to drag when moved again",
    )
    record_parser.add_argument(
        "--control-dt",
        "--control_dt",
        dest="control_dt",
        type=float,
        default=None,
        help="Optional override for the record-loop timestep in seconds; smaller means higher recording/control frequency",
    )
    record_parser.add_argument(
        "--music",
        "--audio",
        dest="music_path",
        default=None,
        help="Optional WAV or MP3 file to play as recording background music",
    )
    record_parser.add_argument(
        "--music-backend",
        default="auto",
        choices=["auto", "g1", "system", "none"],
        help="Audio backend for --music (default: auto)",
    )
    record_parser.add_argument(
        "--music-start",
        default="recording",
        choices=["recording", "command"],
        help="Start music at the recorded motion t=0 or at command start (default: recording)",
    )
    record_parser.add_argument(
        "--music-delay",
        type=float,
        default=0.0,
        help="Seconds to offset music from --music-start; negative values start earlier when possible",
    )
    record_parser.add_argument(
        "--music-stream-name",
        default="music",
        help="G1 audio stream name for background music (default: music)",
    )
    record_parser.add_argument(
        "--music-volume",
        type=int,
        default=DEFAULT_AUDIO_VOLUME,
        help="G1 audio volume for background music, 0-100 (default: 100)",
    )
    record_parser.set_defaults(func=handle_record_motion)

    play_parser = subparsers.add_parser("play-motion", help="Replay an existing motion jsonl")
    play_parser.add_argument("--iface", default=DEFAULT_IFACE, help=iface_help)
    play_parser.add_argument("--path", required=True, help="Motion jsonl path")
    play_parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    play_parser.add_argument(
        "--kp",
        "--profile",
        dest="profile",
        default="upper_playback",
        help="Playback profile name",
    )
    play_parser.add_argument(
        "--debug-takeover",
        action="store_true",
        help="Print takeover diagnostics around the first arm_sdk handoff",
    )
    play_parser.add_argument(
        "--takeover-tau-ff",
        dest="takeover_tau_ff",
        action="store_true",
        help="Use live tau_est as tau_ff during the first arm_sdk takeover hold (default: enabled)",
    )
    play_parser.add_argument(
        "--no-takeover-tau-ff",
        dest="takeover_tau_ff",
        action="store_false",
        help="Disable live tau_est tau_ff support during the first arm_sdk takeover hold",
    )
    play_parser.set_defaults(takeover_tau_ff=True)
    play_parser.add_argument("--dry-run", action="store_true", help="Validate and print the action without connecting")
    play_parser.set_defaults(func=handle_play_motion)

    capture_parser = subparsers.add_parser("capture-snapshot", help="Capture current pose into a snapshot json")
    capture_parser.add_argument("--iface", default=DEFAULT_IFACE, help=iface_help)
    capture_parser.add_argument("--out", required=True, help="Output snapshot path")
    capture_parser.add_argument(
        "--mode",
        default="soft_teach",
        choices=["soft_teach", "current_state"],
        help="Snapshot capture mode: soft_teach hand-guides the arm; current_state saves the live measured pose without taking over arm_sdk",
    )
    capture_parser.add_argument(
        "--group",
        default="upper",
        choices=sorted(ARM_GROUPS.keys()),
        help="Joint group to snapshot",
    )
    capture_parser.add_argument(
        "--settle-time",
        type=float,
        default=0.0,
        help="Optional wait time in seconds before saving a current_state snapshot",
    )
    capture_parser.set_defaults(func=handle_capture_snapshot)

    goto_parser = subparsers.add_parser("goto-snapshot", help="Move to a snapshot pose")
    goto_parser.add_argument("--iface", default=DEFAULT_IFACE, help=iface_help)
    goto_parser.add_argument("--path", required=True, help="Snapshot json path")
    goto_parser.add_argument("--duration", type=float, default=1.5, help="Transition duration in seconds")
    goto_parser.add_argument("--profile", default="upper_hold", help="Playback profile name")
    goto_parser.add_argument(
        "--hold",
        action="store_true",
        help="Keep publishing the final snapshot pose until Ctrl+C before releasing arm_sdk",
    )
    goto_parser.add_argument(
        "--hold-time",
        type=float,
        default=0.0,
        help="Keep publishing the final snapshot pose for this many seconds before releasing arm_sdk",
    )
    goto_parser.add_argument(
        "--snapshot-tau-ff",
        action="store_true",
        help="Use tau_est saved in the snapshot as steady feedforward torque when available",
    )
    goto_parser.add_argument(
        "--debug-takeover",
        action="store_true",
        help="Print takeover diagnostics around the first arm_sdk handoff",
    )
    goto_parser.add_argument(
        "--takeover-tau-ff",
        dest="takeover_tau_ff",
        action="store_true",
        help="Use live tau_est as tau_ff during the first arm_sdk takeover hold (default: enabled)",
    )
    goto_parser.add_argument(
        "--no-takeover-tau-ff",
        dest="takeover_tau_ff",
        action="store_false",
        help="Disable live tau_est tau_ff support during the first arm_sdk takeover hold",
    )
    goto_parser.set_defaults(takeover_tau_ff=True)
    goto_parser.add_argument("--dry-run", action="store_true", help="Validate and print the action without connecting")
    goto_parser.set_defaults(func=handle_goto_snapshot)

    script_parser = subparsers.add_parser("run-script", help="Run a scripted sequence")
    script_parser.add_argument("--iface", default=DEFAULT_IFACE, help=iface_help)
    script_parser.add_argument("--path", required=True, help="Script json path")
    script_parser.add_argument("--profile", default=None, help="Override script playback profile (default: read from script defaults.profile, or upper_playback)")
    script_parser.add_argument(
        "--hand-backend",
        default="none",
        choices=HAND_BACKENDS,
        help="Optional dexterous-hand backend used by script hand steps",
    )
    script_parser.add_argument(
        "--audio-backend",
        default="auto",
        choices=["auto", "g1", "system", "none"],
        help="Audio backend used by script audio steps",
    )
    script_parser.add_argument(
        "--audio-volume",
        type=int,
        default=DEFAULT_AUDIO_VOLUME,
        help="G1 audio volume for script audio steps, 0-100 (default: 100)",
    )
    script_parser.add_argument(
        "--debug-takeover",
        action="store_true",
        help="Print takeover diagnostics for the first snapshot/motion handoff in the script",
    )
    script_parser.add_argument(
        "--takeover-tau-ff",
        dest="takeover_tau_ff",
        action="store_true",
        help="Use live tau_est as tau_ff during the first snapshot/motion takeover hold in the script (default: enabled)",
    )
    script_parser.add_argument(
        "--no-takeover-tau-ff",
        dest="takeover_tau_ff",
        action="store_false",
        help="Disable live tau_est tau_ff support during the first snapshot/motion takeover hold in the script",
    )
    script_parser.set_defaults(takeover_tau_ff=True)
    script_parser.add_argument("--dry-run", action="store_true", help="Validate and print the action without connecting")
    script_parser.set_defaults(func=handle_run_script)

    arm_parser = subparsers.add_parser(
        "arm-action",
        help="Execute a built-in arm action or APP teach action via the firmware RPC service",
    )
    arm_parser.add_argument("--iface", default=DEFAULT_IFACE, help=iface_help)
    arm_group = arm_parser.add_mutually_exclusive_group(required=True)
    arm_group.add_argument("--id", type=int, dest="action_id", help="Built-in action ID (e.g. 17 for clap)")
    arm_group.add_argument("--name", dest="action_name", help="Built-in action name alias (e.g. clap)")
    arm_group.add_argument("--custom", dest="custom_action", help="APP-recorded teach action name (case-sensitive)")
    arm_group.add_argument("--list", action="store_true", help="Query available actions from the robot")
    arm_group.add_argument("--local-list", action="store_true", help="Print local built-in action table (no robot needed)")
    arm_group.add_argument("--sync", action="store_true", help="Sync APP teach actions from the robot into def_motion.json")
    arm_group.add_argument("--stop", action="store_true", help="Stop the currently running custom teach action")
    arm_parser.add_argument(
        "--auto-release",
        action="store_true",
        help="Automatically restore initial arm pose after the action completes",
    )
    arm_parser.add_argument(
        "--hold-time",
        type=float,
        default=2.0,
        help="Wait time in seconds before auto-release (default: 2.0)",
    )
    arm_parser.add_argument(
        "--wait",
        type=float,
        default=None,
        help="Wait time in seconds for non-blocking custom teach actions",
    )
    arm_parser.set_defaults(func=handle_arm_action)

    return parser


def handle_list_ifaces(_args):
    return print_network_interfaces()


def handle_record_motion(args):
    # Delay heavy robot imports so `--help` stays lightweight.
    from .record_modes import record_motion

    return record_motion(
        args.iface,
        args.mode,
        args.out,
        group=args.group,
        auto_hold=args.auto_hold,
        control_dt=args.control_dt,
        music_path=args.music_path,
        music_backend=args.music_backend,
        music_delay=args.music_delay,
        music_start=args.music_start,
        music_stream_name=args.music_stream_name,
        music_volume=args.music_volume,
    )


def handle_play_motion(args):
    from .motion_io import load_motion, play_motion

    trajectory = load_motion(args.path)
    if args.dry_run:
        # Dry-run only validates the file and prints the execution plan.
        play_motion(
            None,
            trajectory,
            profile_name=args.profile,
            speed=args.speed,
            dry_run=True,
            release=False,
            debug_takeover=args.debug_takeover,
            takeover_tau_ff=args.takeover_tau_ff,
        )
        return 0

    from .robot_io import RobotSession

    session = RobotSession(iface=args.iface, enable_pub=True)
    play_motion(
        session,
        trajectory,
        profile_name=args.profile,
        speed=args.speed,
        dry_run=False,
        release=True,
        debug_takeover=args.debug_takeover,
        takeover_tau_ff=args.takeover_tau_ff,
    )
    return 0


def handle_capture_snapshot(args):
    from .robot_io import RobotSession
    from .snapshot_io import capture_current_snapshot, capture_snapshot

    if args.mode == "soft_teach":
        # Interactive hand-guiding takes over arm_sdk.
        session = RobotSession(iface=args.iface, enable_pub=True)
        capture_snapshot(session, args.out, group=args.group)
        return 0

    # current_state mode only reads lowstate so it preserves the robot's live controller pose.
    session = RobotSession(iface=args.iface, enable_pub=False)
    capture_current_snapshot(session, args.out, group=args.group, settle_time=args.settle_time)
    return 0


def handle_goto_snapshot(args):
    from .snapshot_io import goto_snapshot, load_snapshot

    snapshot = load_snapshot(args.path)
    if args.dry_run:
        goto_snapshot(
            None,
            snapshot,
            duration=args.duration,
            profile_name=args.profile,
            dry_run=True,
            release=False,
            debug_takeover=args.debug_takeover,
            takeover_tau_ff=args.takeover_tau_ff,
            hold=args.hold,
            hold_time=args.hold_time,
            snapshot_tau_ff=args.snapshot_tau_ff,
        )
        return 0

    from .robot_io import RobotSession

    session = RobotSession(iface=args.iface, enable_pub=True)
    goto_snapshot(
        session,
        snapshot,
        duration=args.duration,
        profile_name=args.profile,
        dry_run=False,
        release=True,
        debug_takeover=args.debug_takeover,
        takeover_tau_ff=args.takeover_tau_ff,
        hold=args.hold,
        hold_time=args.hold_time,
        snapshot_tau_ff=args.snapshot_tau_ff,
    )
    return 0


def handle_run_script(args):
    from .script_runner import run_script

    if args.dry_run:
        run_script(
            None,
            args.path,
            cli_profile=args.profile,
            dry_run=True,
            release=False,
            hand_backend_name=args.hand_backend,
            debug_takeover=args.debug_takeover,
            takeover_tau_ff=args.takeover_tau_ff,
            audio_backend=args.audio_backend,
            audio_volume=args.audio_volume,
        )
        return 0

    from .hand_adapters import create_hand_adapter
    from .robot_io import RobotSession

    session = RobotSession(iface=args.iface, enable_pub=True)
    hand_adapter = create_hand_adapter(args.hand_backend)
    try:
        run_script(
            session,
            args.path,
            cli_profile=args.profile,
            dry_run=False,
            release=True,
            hand_adapter=hand_adapter,
            hand_backend_name=args.hand_backend,
            debug_takeover=args.debug_takeover,
            takeover_tau_ff=args.takeover_tau_ff,
            audio_backend=args.audio_backend,
            audio_volume=args.audio_volume,
        )
        return 0
    finally:
        if hand_adapter is not None:
            hand_adapter.close()


def handle_arm_action(args):
    from .arm_action import (
        BUILTIN_ACTION_BY_NAME,
        execute_builtin_action,
        execute_custom_action,
        list_actions,
        list_builtin_actions,
        stop_custom_action,
        sync_actions,
    )

    if args.local_list:
        list_builtin_actions()
        return 0

    if args.list:
        return list_actions(args.iface)

    if args.sync:
        return sync_actions(args.iface)

    if args.stop:
        return stop_custom_action(args.iface)

    if args.custom_action:
        return execute_custom_action(args.iface, args.custom_action, wait=args.wait)

    # Resolve action ID from --id or --name.
    action_id = args.action_id
    if action_id is None and args.action_name:
        action_id = BUILTIN_ACTION_BY_NAME.get(args.action_name)
        if action_id is None:
            print(f"未知的内置动作名称: {args.action_name}")
            print("使用 --local-list 查看可用名称")
            return 1

    return execute_builtin_action(
        args.iface,
        action_id,
        auto_release=args.auto_release,
        hold_time=args.hold_time,
    )


def main(argv=None):
    """Parse CLI arguments and return a numeric process exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args) or 0
