"""Helpers for selecting the robot-facing network interface."""

from __future__ import annotations

import ipaddress
import json
import os
import platform
import socket
import subprocess
from dataclasses import dataclass
from functools import lru_cache


AUTO_IFACE = "auto"
LEGACY_DEFAULT_IFACE = "enP8p1s0"
IFACE_ENV_VAR = "G1_TEACH_IFACE"


@dataclass
class InterfaceCandidate:
    name: str
    ipv4: list[str]
    score: int
    reasons: list[str]


def _run_command(cmd):
    try:
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
        )
    except OSError:
        return ""
    return (result.stdout or "").strip()


def _normalize_json_sequence(payload):
    if not payload or payload == "null":
        return []
    try:
        parsed = json.loads(payload)
    except json.JSONDecodeError:
        return []
    if isinstance(parsed, list):
        return parsed
    if isinstance(parsed, dict):
        return [parsed]
    return []


def _list_windows_ipv4():
    script = (
        "[Console]::OutputEncoding = [System.Text.Encoding]::UTF8;"
        "$items = Get-NetIPAddress -AddressFamily IPv4 "
        "| Where-Object { $_.IPAddress -ne '127.0.0.1' } "
        "| Select-Object InterfaceAlias,IPAddress,PrefixLength;"
        "$items | ConvertTo-Json -Compress"
    )
    payload = _run_command(["powershell", "-NoProfile", "-Command", script])
    grouped = {}
    for item in _normalize_json_sequence(payload):
        name = str(item.get("InterfaceAlias", "")).strip()
        ip = str(item.get("IPAddress", "")).strip()
        if not name or not ip:
            continue
        grouped.setdefault(name, []).append(ip)
    return grouped


def _list_linux_ipv4():
    payload = _run_command(["ip", "-j", "-4", "addr", "show"])
    grouped = {}
    for item in _normalize_json_sequence(payload):
        name = str(item.get("ifname", "")).strip()
        if not name:
            continue
        grouped[name] = [
            str(addr.get("local", "")).strip()
            for addr in item.get("addr_info", [])
            if str(addr.get("local", "")).strip()
        ]
    return grouped


def _list_interface_names():
    try:
        return [name for _, name in socket.if_nameindex()]
    except OSError:
        return []


def _list_ipv4_by_interface():
    system = platform.system().lower()
    if system == "windows":
        grouped = _list_windows_ipv4()
    else:
        grouped = _list_linux_ipv4()

    for name in _list_interface_names():
        grouped.setdefault(name, [])
    return grouped


def _looks_private(ip_text):
    try:
        ip = ipaddress.ip_address(ip_text)
    except ValueError:
        return False
    return bool(ip.is_private)


def _score_interface(name, ipv4):
    lname = name.lower()
    score = 0
    reasons = []

    if ipv4:
        score += 20
        reasons.append("has IPv4")
    if any(_looks_private(ip) for ip in ipv4):
        score += 20
        reasons.append("private IPv4")
    if any(ip.startswith("192.168.123.") for ip in ipv4):
        score += 25
        reasons.append("matches common Unitree subnet")
    if any(ip.startswith("192.168.") for ip in ipv4):
        score += 10
        reasons.append("LAN-style subnet")
    if any(ip.startswith("169.254.") for ip in ipv4):
        score -= 40
        reasons.append("link-local only")

    wired_tokens = ("ethernet", "eth", "en", "eno", "enp", "enx", "以太网")
    if any(token in lname for token in wired_tokens):
        score += 12
        reasons.append("wired-style name")

    wifi_tokens = ("wifi", "wi-fi", "wireless", "wlan", "wl", "无线")
    if any(token in lname for token in wifi_tokens):
        score -= 8
        reasons.append("wireless-style name")

    virtual_tokens = (
        "loopback",
        "docker",
        "veth",
        "vmware",
        "virtualbox",
        "hyper-v",
        "tailscale",
        "zerotier",
        "vpn",
        "bluetooth",
        "wsl",
    )
    if any(token in lname for token in virtual_tokens) or lname == "lo":
        score -= 30
        reasons.append("virtual/overlay adapter")

    if name == LEGACY_DEFAULT_IFACE:
        score += 5
        reasons.append("legacy default name")

    return score, reasons


@lru_cache(maxsize=1)
def list_network_interfaces():
    candidates = []
    for name, ipv4 in sorted(_list_ipv4_by_interface().items()):
        score, reasons = _score_interface(name, ipv4)
        candidates.append(InterfaceCandidate(name=name, ipv4=list(ipv4), score=score, reasons=reasons))
    candidates.sort(key=lambda item: (item.score, len(item.ipv4), item.name == LEGACY_DEFAULT_IFACE), reverse=True)
    return tuple(candidates)


def describe_network_interfaces():
    lines = []
    for item in list_network_interfaces():
        ips = ", ".join(item.ipv4) if item.ipv4 else "-"
        why = ", ".join(item.reasons) if item.reasons else "no IPv4 details"
        lines.append(f"{item.name}: ipv4=[{ips}] score={item.score} ({why})")
    return lines


@lru_cache(maxsize=1)
def _resolve_auto_network_interface():
    env_iface = os.getenv(IFACE_ENV_VAR, "").strip()
    if env_iface:
        return env_iface, f"{IFACE_ENV_VAR}={env_iface}"

    candidates = list_network_interfaces()
    if not candidates:
        raise RuntimeError(
            "no network interfaces detected; pass --iface <name> or set "
            f"{IFACE_ENV_VAR}=<name>"
        )

    for item in candidates:
        if item.score > -20 and item.ipv4:
            ips = ", ".join(item.ipv4)
            return item.name, f"auto-selected {item.name} ({ips})"

    fallback = candidates[0]
    ips = ", ".join(fallback.ipv4) if fallback.ipv4 else "no IPv4"
    return fallback.name, f"fallback-selected {fallback.name} ({ips})"


def resolve_network_interface(iface=AUTO_IFACE, verbose=False):
    explicit = str(iface or "").strip()
    if explicit and explicit.lower() != AUTO_IFACE:
        return explicit

    resolved, reason = _resolve_auto_network_interface()
    if verbose:
        print(f"[NET] {reason}")
    return resolved


def print_network_interfaces():
    candidates = list_network_interfaces()
    if not candidates:
        print("[NET] no interfaces found")
        return 1

    print("[NET] detected interfaces:")
    for line in describe_network_interfaces():
        print(f"  {line}")

    best = resolve_network_interface(AUTO_IFACE, verbose=False)
    print(f"[NET] recommended interface: {best}")
    print(f"[NET] override with --iface <name> or {IFACE_ENV_VAR}=<name>")
    return 0
