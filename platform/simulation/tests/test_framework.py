#!/usr/bin/env python3
"""
Autonomous Test Framework for WRO 2026 Navigator

Modes:
  report  — Run navigator on scenarios, parse logs, produce markdown report.
  train   — After each failed scenario, apply heuristic param adjustments and
             retry until pass or max-retries exceeded.

Gazebo is managed manually. This framework only manages the navigator subprocess.

Usage:
    python3 test_framework.py --mode report --challenge open --scenarios 0-9 \\
        --laps 3 --timeout 120 --output report.md

    python3 test_framework.py --mode train --challenge open --scenarios 0-4 \\
        --laps 3 --timeout 120 --max-retries 5 --output train_report.md
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from argparse import Namespace
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).parent
PARAMS_FILE = SCRIPTS_DIR / "navigator_params.json"

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class EscapeEvent:
    pos: tuple[float, float]
    corner: str
    direction: str  # 'CW', 'CCW', or ''
    forward_dist: float
    count: int  # escape number at that location


@dataclass
class LoopEvent:
    pos: tuple[float, float]
    corner: str
    repeat_count: int


@dataclass
class StuckEvent:
    pos: tuple[float, float]
    corner: str
    duration_s: float


@dataclass
class SkipEvent:
    waypoint_id: int
    pos: tuple[float, float]
    reason: str


@dataclass
class ScenarioResult:
    scenario_id: int
    challenge_type: str
    corridor_widths: dict[str, Any]
    laps_completed: int
    target_laps: int
    timed_out: bool
    duration_s: float
    escape_events: list[EscapeEvent] = field(default_factory=list)
    loop_events: list[LoopEvent] = field(default_factory=list)
    stuck_events: list[StuckEvent] = field(default_factory=list)
    waypoint_skips: list[SkipEvent] = field(default_factory=list)
    params_used: dict[str, Any] = field(default_factory=dict)
    passed: bool = False


# ---------------------------------------------------------------------------
# LogParser
# ---------------------------------------------------------------------------


class LogParser:
    """Parse navigator stdout/stderr for structured events."""

    # Patterns
    _RE_CRITICAL = re.compile(
        r"CRITICAL #(\d+):.*?F=([\d.]+)m.*?"
        r"(?:pos=\(([\d. -]+),([\d. -]+)\))?"
        r".*?(RIGHT/CW|LEFT/CCW)?",
    )
    _RE_WALL_CONTACT = re.compile(
        r"WALL CONTACT: F=([\d.]+)m",
    )
    _RE_OBSTACLE = re.compile(
        r"OBSTACLE CONTACT",
    )
    _RE_STUCK = re.compile(
        r"STUCK DETECTED \(([\d. -]+),([\d. -]+)\) for ([\d.]+)s",
    )
    _RE_SKIP = re.compile(
        r"(?:Skipping|Skipped) waypoint (\d+)",
    )
    _RE_CRITICAL_LOOP = re.compile(
        r"CRITICAL LOOP \((\d+) escapes near \(([\d. -]+),([\d. -]+)\)\)",
    )
    _RE_OBSTACLE_LOOP = re.compile(
        r"Obstacle loop detected \((\d+) escapes near \(([\d. -]+),([\d. -]+)\)\)",
    )
    _RE_LAP = re.compile(
        r"(?:LAP COMPLETE|Lap (\d+) complete|Completed (\d+) laps)",
    )
    _RE_NAV_POS = re.compile(
        r"NAV: pos=\(([\d. -]+),([\d. -]+)\)",
    )
    _RE_ESCAPE_DIR = re.compile(
        r"escape: (CW|CCW)|→ (RIGHT/CW|LEFT/CCW)",
    )

    def __init__(self) -> None:
        self.escape_events: list[EscapeEvent] = []
        self.loop_events: list[LoopEvent] = []
        self.stuck_events: list[StuckEvent] = []
        self.waypoint_skips: list[SkipEvent] = []
        self.laps_completed = 0
        self._last_pos: tuple[float, float] = (0.0, 0.0)
        self._pending_skip_wp: int | None = None

    def feed(self, line: str, classifier: CornerClassifier) -> None:
        """Process a single log line."""
        line = line.strip()

        # Track current position from NAV lines
        m = self._RE_NAV_POS.search(line)
        if m:
            self._last_pos = (float(m.group(1)), float(m.group(2)))

        # Lap completion
        m = self._RE_LAP.search(line)
        if m:
            self.laps_completed += 1
            return

        # Critical escape
        m = self._RE_CRITICAL.search(line)
        if m:
            count = int(m.group(1))
            fwd = float(m.group(2))
            if m.group(3) and m.group(4):
                pos = (float(m.group(3)), float(m.group(4)))
            else:
                pos = self._last_pos
            direction = ""
            if m.group(5):
                direction = "CW" if "CW" in m.group(5) else "CCW"
            corner = classifier.classify(pos)
            self.escape_events.append(
                EscapeEvent(
                    pos=pos,
                    corner=corner,
                    direction=direction,
                    forward_dist=fwd,
                    count=count,
                )
            )
            return

        # Wall contact (alternate log format)
        m = self._RE_WALL_CONTACT.search(line)
        if m and not self._RE_CRITICAL.search(line):
            fwd = float(m.group(1))
            pos = self._last_pos
            # Determine direction from same line if present
            dm = self._RE_ESCAPE_DIR.search(line)
            direction = ""
            if dm:
                raw = dm.group(1) or dm.group(2) or ""
                direction = "CW" if "CW" in raw else ("CCW" if "CCW" in raw else "")
            corner = classifier.classify(pos)
            self.escape_events.append(
                EscapeEvent(
                    pos=pos,
                    corner=corner,
                    direction=direction,
                    forward_dist=fwd,
                    count=len(self.escape_events) + 1,
                )
            )
            return

        # Critical loop
        m = self._RE_CRITICAL_LOOP.search(line)
        if m:
            repeat = int(m.group(1))
            pos = (float(m.group(2)), float(m.group(3)))
            corner = classifier.classify(pos)
            self.loop_events.append(
                LoopEvent(
                    pos=pos,
                    corner=corner,
                    repeat_count=repeat,
                )
            )
            return

        # Obstacle loop
        m = self._RE_OBSTACLE_LOOP.search(line)
        if m:
            repeat = int(m.group(1))
            pos = (float(m.group(2)), float(m.group(3)))
            corner = classifier.classify(pos)
            self.loop_events.append(
                LoopEvent(
                    pos=pos,
                    corner=corner,
                    repeat_count=repeat,
                )
            )
            return

        # Stuck detection
        m = self._RE_STUCK.search(line)
        if m:
            pos = (float(m.group(1)), float(m.group(2)))
            duration = float(m.group(3))
            corner = classifier.classify(pos)
            self.stuck_events.append(
                StuckEvent(
                    pos=pos,
                    corner=corner,
                    duration_s=duration,
                )
            )
            return

        # Waypoint skip
        m = self._RE_SKIP.search(line)
        if m:
            wp_id = int(m.group(1))
            pos = self._last_pos
            reason = "loop" if "loop" in line.lower() else "passed"
            self.waypoint_skips.append(
                SkipEvent(
                    waypoint_id=wp_id,
                    pos=pos,
                    reason=reason,
                )
            )


# ---------------------------------------------------------------------------
# CornerClassifier
# ---------------------------------------------------------------------------


class CornerClassifier:
    """Classify (x, y) positions into track corners or corridor segments."""

    def __init__(self, corridor_widths: dict) -> None:
        """
        corridor_widths: dict with keys 'north', 'south', 'east', 'west',
                         each having 'width_mm'.
        """
        track_max = 3.0
        n_w = corridor_widths.get("north", {}).get("width_mm", 600) / 1000.0
        s_w = corridor_widths.get("south", {}).get("width_mm", 600) / 1000.0
        e_w = corridor_widths.get("east", {}).get("width_mm", 600) / 1000.0
        w_w = corridor_widths.get("west", {}).get("width_mm", 600) / 1000.0

        # Interior wall positions (mirror generate_training_data.py geometry)
        # Interior wall is placed at corridor_width from the outer edge
        self.north_interior_y = track_max - n_w - 0.05
        self.south_interior_y = s_w + 0.05
        self.east_interior_x = track_max - e_w - 0.05
        self.west_interior_x = w_w + 0.05

    def classify(self, pos: tuple[float, float]) -> str:
        x, y = pos
        in_east = x > self.east_interior_x - 0.5
        in_west = x < self.west_interior_x + 0.5
        in_north = y > self.north_interior_y - 0.5
        in_south = y < self.south_interior_y + 0.5

        if in_east and in_north:
            return "NE"
        if in_west and in_north:
            return "NW"
        if in_east and in_south:
            return "SE"
        if in_west and in_south:
            return "SW"

        # Corridor: determine by proximity
        dists = {
            "N": abs(y - 3.0),
            "S": abs(y - 0.0),
            "E": abs(x - 3.0),
            "W": abs(x - 0.0),
        }
        return min(dists, key=lambda direction: dists[direction])


# ---------------------------------------------------------------------------
# ParameterTuner
# ---------------------------------------------------------------------------


class ParameterTuner:
    """Apply heuristic rules to suggest parameter patches for failed scenarios."""

    RULES = [
        "R1_gpu_artifact",
        "R2_corner_loop",
        "R3_frequent_stuck",
        "R4_real_wall_crash",
        "R5_waypoint_skips",
    ]

    def suggest(self, result: ScenarioResult) -> dict | None:
        """Return a param patch dict, or None if no rule fires."""
        params = dict(result.params_used)
        patches: dict = {}

        # R1: GPU artifact — escapes at corner with very low forward dist and
        #     no prior real-wall contact (count=1 escapes only)
        artifact_escapes = [
            e for e in result.escape_events if e.forward_dist < 0.08 and e.count == 1
        ]
        if artifact_escapes:
            cur = params.get("fwd_critical_lidar_threshold", 2)
            if cur < 4:
                patches["fwd_critical_lidar_threshold"] = cur + 1
                patches["_rule"] = "R1: GPU artifact → fwd_critical_lidar_threshold+1"

        # R2: Corner loops — repeated critical loop at same position
        bad_loops = [e for e in result.loop_events if e.repeat_count >= 3]
        if bad_loops and "fwd_critical_lidar_threshold" not in patches:
            cur = params.get("critical_repeat_radius", 0.25)
            if cur < 0.50:
                patches["critical_repeat_radius"] = round(cur + 0.05, 3)
                patches["_rule"] = "R2: Corner loop → critical_repeat_radius+0.05"

        # R3: Frequent stuck
        if (
            len(result.stuck_events) > 3
            and "escape_duration_base" not in patches
            and "_rule" not in patches
        ):
            cur = params.get("escape_duration_base", 10)
            if cur < 20:
                patches["escape_duration_base"] = cur + 2
                patches["_rule"] = "R3: Frequent stuck → escape_duration_base+2"

        # R4: Many real wall crashes in narrow corridors
        narrow_corridors = any(
            w.get("width_mm", 1000) < 650 for w in result.corridor_widths.values()
        )
        if len(result.escape_events) > 10 and narrow_corridors and "_rule" not in patches:
            cur = params.get("critical_distance", 0.07)
            if cur < 0.12:
                patches["critical_distance"] = round(cur + 0.01, 3)
                patches["_rule"] = "R4: Real wall crashes → critical_distance+0.01"

        # R5: Too many waypoint skips
        if len(result.waypoint_skips) > 5 and "_rule" not in patches:
            cur = params.get("waypoint_threshold", 0.20)
            if cur > 0.10:
                patches["waypoint_threshold"] = round(cur - 0.02, 3)
                patches["_rule"] = "R5: Many skips → waypoint_threshold-0.02"

        if not patches:
            return None
        return patches


# ---------------------------------------------------------------------------
# Reporter
# ---------------------------------------------------------------------------


class Reporter:
    """Generate markdown report from scenario results."""

    def write_markdown(
        self,
        results: list[ScenarioResult],
        output_path: str,
        param_history: list[dict[str, Any]] | None = None,
    ) -> None:
        lines = ["# WRO 2026 Navigator Test Report\n"]
        lines.append(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        lines.append(f"Scenarios: {len(results)}\n")

        # Summary table
        lines.append("\n## Summary\n")
        lines.append("| Scenario | Challenge | Laps | Time (s) | Timeout | Result |")
        lines.append("|----------|-----------|------|----------|---------|--------|")
        for r in results:
            status = "PASS" if r.passed else ("TIMEOUT" if r.timed_out else "FAIL")
            lines.append(
                f"| {r.scenario_id:04d} | {r.challenge_type} | "
                f"{r.laps_completed}/{r.target_laps} | {r.duration_s:.1f} | "
                f"{'yes' if r.timed_out else 'no'} | **{status}** |",
            )

        # Per-scenario sections
        for r in results:
            lines.append(f"\n---\n\n## Scenario {r.scenario_id:04d}\n")

            # Corridor widths
            lines.append("### Corridor Widths\n")
            lines.append("| Corridor | Width (mm) |")
            lines.append("|----------|------------|")
            for side, info in r.corridor_widths.items():
                lines.append(f"| {side.capitalize()} | {info.get('width_mm', '?')} |")

            # Params used
            lines.append("\n### Parameters Used\n")
            lines.append("```json")
            lines.append(
                json.dumps(
                    {k: v for k, v in r.params_used.items() if not k.startswith("_")}, indent=2
                )
            )
            lines.append("```")

            # Escape events
            if r.escape_events:
                lines.append("\n### Escape Events\n")
                lines.append("| # | Corner | Direction | F (m) | Position |")
                lines.append("|---|--------|-----------|-------|----------|")
                for i, e in enumerate(r.escape_events, 1):
                    lines.append(
                        f"| {i} | {e.corner} | {e.direction or '-'} | "
                        f"{e.forward_dist:.3f} | ({e.pos[0]:.2f}, {e.pos[1]:.2f}) |",
                    )

            # Loop events
            if r.loop_events:
                lines.append("\n### Loop Events\n")
                lines.append("| Corner | Repeat Count | Position |")
                lines.append("|--------|--------------|----------|")
                for e in r.loop_events:
                    lines.append(
                        f"| {e.corner} | {e.repeat_count} | ({e.pos[0]:.2f}, {e.pos[1]:.2f}) |",
                    )

            # Stuck events
            if r.stuck_events:
                lines.append("\n### Stuck Events\n")
                lines.append("| Corner | Duration (s) | Position |")
                lines.append("|--------|--------------|----------|")
                for e in r.stuck_events:
                    lines.append(
                        f"| {e.corner} | {e.duration_s:.1f} | ({e.pos[0]:.2f}, {e.pos[1]:.2f}) |",
                    )

            # Waypoint skips
            if r.waypoint_skips:
                lines.append("\n### Waypoint Skips\n")
                lines.append("| WP | Reason | Position |")
                lines.append("|----|--------|----------|")
                for e in r.waypoint_skips:
                    lines.append(
                        f"| {e.waypoint_id} | {e.reason} | ({e.pos[0]:.2f}, {e.pos[1]:.2f}) |",
                    )

        # Parameter evolution (train mode)
        if param_history:
            lines.append("\n---\n\n## Parameter Evolution (Train Mode)\n")
            lines.append("| Attempt | Rule | Parameter | Old | New |")
            lines.append("|---------|------|-----------|-----|-----|")
            for entry in param_history:
                lines.append(
                    f"| {entry['attempt']} | {entry.get('rule', '-')} | "
                    f"{entry.get('param', '-')} | {entry.get('old', '-')} | "
                    f"{entry.get('new', '-')} |",
                )

            # Final recommended params
            lines.append("\n### Recommended `navigator_params.json`\n")
            lines.append("```json")
            if results:
                final_params = {
                    k: v for k, v in results[-1].params_used.items() if not k.startswith("_")
                }
                lines.append(json.dumps(final_params, indent=2))
            lines.append("```")

        content = "\n".join(lines) + "\n"
        with open(output_path, "w") as f:
            f.write(content)
        print(f"Report written to: {output_path}")


# ---------------------------------------------------------------------------
# Scenario discovery helpers
# ---------------------------------------------------------------------------


def parse_scenario_range(spec: str) -> list[int]:
    """Parse '0-9' or '0,2,5' into a list of ints."""
    ids = []
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = part.split("-", 1)
            ids.extend(range(int(lo), int(hi) + 1))
        else:
            ids.append(int(part))
    return sorted(set(ids))


def find_metadata(scenario_id: int, challenge: str, search_dirs: list[Path]) -> Path | None:
    """Locate metadata JSON for a given scenario id."""
    name = f"scenario_{scenario_id:04d}_metadata.json"
    for d in search_dirs:
        p = d / name
        if p.exists():
            return p
    return None


def challenge_search_dirs(challenge: str) -> list[Path]:
    """Return candidate metadata directories based on challenge type."""
    training_root = SCRIPTS_DIR / "training_data"
    challenges = ["open", "obstacles"] if challenge == "all" else [challenge]
    dirs = []
    for ch in challenges:
        dirs.append(training_root / ch / "scenarios")
    # Fallbacks
    dirs += [SCRIPTS_DIR, SCRIPTS_DIR.parent / "worlds", SCRIPTS_DIR.parent, Path.cwd()]
    return dirs


# ---------------------------------------------------------------------------
# Navigator subprocess management
# ---------------------------------------------------------------------------


def launch_navigator(metadata_path: Path, laps: int) -> subprocess.Popen:
    """Start navigator subprocess, returning the Popen object."""
    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "track_navigator.py"),
        "--metadata",
        str(metadata_path),
        "--laps",
        str(laps),
    ]
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )


def kill_subprocess(proc: subprocess.Popen) -> None:
    """Terminate subprocess cleanly, then SIGKILL if needed."""
    if proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


# ---------------------------------------------------------------------------
# TestFramework
# ---------------------------------------------------------------------------


def load_params() -> dict[str, Any]:
    if PARAMS_FILE.exists():
        with open(PARAMS_FILE) as f:
            return json.load(f)
    return {}


def save_params(params: dict[str, Any]) -> None:
    with open(PARAMS_FILE, "w") as f:
        json.dump(params, f, indent=2)
        f.write("\n")


class TestFramework:
    def __init__(self, args: Namespace) -> None:
        self.args = args
        self.tuner = ParameterTuner()
        self.reporter = Reporter()
        self.search_dirs = challenge_search_dirs(args.challenge)

    # ------------------------------------------------------------------
    # Scenario runner
    # ------------------------------------------------------------------

    def run_scenario(
        self, scenario_id: int, metadata_path: Path, prompt_prefix: str = ""
    ) -> ScenarioResult:
        """Prompt user, launch navigator, collect result."""
        prompt = (
            f"{prompt_prefix}Press Enter when Gazebo is ready for scenario {scenario_id:04d} > "
        )
        input(prompt)

        # Load current params snapshot
        params_snapshot = {k: v for k, v in load_params().items() if not k.startswith("_")}

        # Load metadata for corridor widths
        with open(metadata_path) as f:
            metadata = json.load(f)
        corridor_widths = metadata.get("corridor_widths", {})
        challenge_type = metadata.get("challenge_type", "open")

        classifier = CornerClassifier(corridor_widths)
        parser = LogParser()

        proc = launch_navigator(metadata_path, self.args.laps)
        start_time = time.time()
        timed_out = False
        stdout = proc.stdout
        assert stdout is not None, "navigator stdout not captured"

        try:
            while proc.poll() is None:
                elapsed = time.time() - start_time
                if elapsed >= self.args.timeout:
                    print(f"  [timeout after {elapsed:.0f}s]")
                    timed_out = True
                    break

                # Read lines with a short timeout via readline
                line = stdout.readline()
                if line:
                    print(f"  NAV: {line}", end="")
                    parser.feed(line, classifier)

                    # Early exit if all laps done
                    if parser.laps_completed >= self.args.laps:
                        # Drain remaining output briefly
                        time.sleep(0.5)
                        while True:
                            line_tail = stdout.readline()
                            if not line_tail:
                                break
                            parser.feed(line_tail, classifier)
                        break
        finally:
            kill_subprocess(proc)

        duration = time.time() - start_time
        passed = parser.laps_completed >= self.args.laps

        return ScenarioResult(
            scenario_id=scenario_id,
            challenge_type=challenge_type,
            corridor_widths=corridor_widths,
            laps_completed=parser.laps_completed,
            target_laps=self.args.laps,
            timed_out=timed_out,
            duration_s=duration,
            escape_events=parser.escape_events,
            loop_events=parser.loop_events,
            stuck_events=parser.stuck_events,
            waypoint_skips=parser.waypoint_skips,
            params_used=params_snapshot,
            passed=passed,
        )

    # ------------------------------------------------------------------
    # Report mode
    # ------------------------------------------------------------------

    def run_report(self, scenario_ids: list[int]) -> list[ScenarioResult]:
        results = []
        for sid in scenario_ids:
            metadata = find_metadata(sid, self.args.challenge, self.search_dirs)
            if metadata is None:
                print(f"WARNING: metadata not found for scenario {sid:04d}, skipping.")
                continue
            print(f"\n=== Scenario {sid:04d} ===")
            result = self.run_scenario(sid, metadata)
            results.append(result)
            status = "PASS" if result.passed else ("TIMEOUT" if result.timed_out else "FAIL")
            print(
                f"  → {status}: {result.laps_completed}/{result.target_laps} laps, "
                f"{result.duration_s:.1f}s, "
                f"{len(result.escape_events)} escapes, "
                f"{len(result.loop_events)} loops"
            )
        return results

    # ------------------------------------------------------------------
    # Train mode
    # ------------------------------------------------------------------

    def run_train(self, scenario_ids: list[int]) -> tuple[list[ScenarioResult], list[dict]]:
        all_results: list[ScenarioResult] = []
        param_history: list[dict] = []

        for sid in scenario_ids:
            metadata = find_metadata(sid, self.args.challenge, self.search_dirs)
            if metadata is None:
                print(f"WARNING: metadata not found for scenario {sid:04d}, skipping.")
                continue

            print(f"\n=== Scenario {sid:04d} (train mode) ===")
            attempt = 0

            while attempt < self.args.max_retries:
                prompt = f"  [attempt {attempt + 1}/{self.args.max_retries}] "
                result = self.run_scenario(sid, metadata, prompt_prefix=prompt)
                all_results.append(result)

                status = "PASS" if result.passed else ("TIMEOUT" if result.timed_out else "FAIL")
                print(
                    f"  → {status}: {result.laps_completed}/{result.target_laps} laps, "
                    f"{result.duration_s:.1f}s"
                )

                if result.passed:
                    break

                patch = self.tuner.suggest(result)
                if patch is None:
                    print("  No applicable tuning rule. Moving to next scenario.")
                    break

                rule = patch.pop("_rule", "")
                print(f"  Applying: {rule}")

                # Apply patch to params file
                current = load_params()
                for k, v in patch.items():
                    old_val = current.get(k, "?")
                    current[k] = v
                    param_history.append(
                        {
                            "attempt": attempt + 1,
                            "scenario": sid,
                            "rule": rule,
                            "param": k,
                            "old": old_val,
                            "new": v,
                        }
                    )
                    print(f"    {k}: {old_val} → {v}")
                save_params(current)

                attempt += 1

        return all_results, param_history

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        scenario_ids = parse_scenario_range(self.args.scenarios)
        print(
            f"Mode: {self.args.mode} | Challenge: {self.args.challenge} | "
            f"Scenarios: {scenario_ids} | Laps: {self.args.laps} | "
            f"Timeout: {self.args.timeout}s"
        )

        if self.args.mode == "report":
            results = self.run_report(scenario_ids)
            self.reporter.write_markdown(results, self.args.output)
        else:  # train
            results, param_history = self.run_train(scenario_ids)
            self.reporter.write_markdown(results, self.args.output, param_history)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Autonomous test framework for WRO 2026 navigator",
    )
    parser.add_argument("--mode", choices=["report", "train"], required=True)
    parser.add_argument("--challenge", choices=["open", "obstacles", "all"], default="open")
    parser.add_argument(
        "--scenarios", type=str, default="0", help='Range like "0-9" or list like "0,2,5"'
    )
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument(
        "--timeout", type=int, default=120, help="Seconds before navigator subprocess is killed"
    )
    parser.add_argument(
        "--max-retries", type=int, default=5, help="Train mode: max retry attempts per scenario"
    )
    parser.add_argument(
        "--output", type=str, default="report.md", help="Output markdown report file"
    )
    args = parser.parse_args()

    framework = TestFramework(args)
    framework.run()


if __name__ == "__main__":
    main()
