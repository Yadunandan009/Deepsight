#!/usr/bin/env python3
"""G5: how often the SLAM pose bridge's gate and quarantine acted, per mission
of the veto/symmetry pre-registration.

    python3 eval/bridge_events.py      # after eval/prereg_score.py has run

Each flight's bridge log is found with nothing hand-picked:
  ~/.ros/log/<launch dir>/launch.log   the launch that started closest before
                                        the bag (within 900 s) gives the
                                        bridge's pid, start and death times;
  ~/.ros/log/python3_<pid>_*.log        is that bridge's own log.

Coverage check: the bridge process was alive over the whole mission (launch
start before the bag, death after mission end), its log opens with
`reference_odom=/bluerov2/odometry/dr` (the PREREG's dr_reference:=true), and
it has messages inside the mission.

Counted, before the primary endpoint's cutoff (RETURN_HOME start) and to
mission end, every message class the bridge can print (slam_pose_bridge.py):
  lock            SLAM alignment solved
  refit           SLAM alignment refit
  gate_reject     SLAM pose rejected ... streak=N   (throttled to one per
                  2 s, so also the number of rejection EPISODES: a run of
                  lines with increasing streak is one episode)
  stale_realign   SLAM alignment stale ... re-aligning from scratch
  quarantine      SLAM QUARANTINED
  jump_reset      SLAM jump detected ... resetting alignment
  jump_new_map    jump before this map was ever aligned
  map_crash       Map point crash detected
  fit_rejected    Fit rejected: scale ...
  refit_rejected  Refit rejected / Refit drift ... exceeds
  probation       alignment on probation (throttled to one per 5 s)
"""
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import prereg_score as ps   # noqa: E402

LOGDIR = os.path.expanduser("~/.ros/log")
OUT_JSON = os.path.join(HERE, "figures", "prereg_bridge_events.json")
OUT_MD = os.path.join(ROOT, "docs", "paper", "prereg_bridge_events.md")
CLASSES = [("lock", "SLAM alignment solved"), ("refit", "SLAM alignment refit"),
           ("gate_reject", "SLAM pose rejected"), ("stale_realign", "SLAM alignment stale"),
           ("quarantine", "SLAM QUARANTINED"), ("jump_reset", "SLAM jump detected"),
           ("jump_new_map", "jump before this map was ever aligned"),
           ("map_crash", "Map point crash detected"), ("fit_rejected", "Fit rejected"),
           ("refit_rejected", "Refit rejected"), ("refit_rejected", "Refit drift"),
           ("probation", "alignment on probation")]
TS = re.compile(r"\[(\d{10}\.\d+)\]")
STREAK = re.compile(r"streak=(\d+)")


def first_truth_time(bag):
    from mcap_ros2.reader import read_ros2_messages
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[ps.sr.T_TRUTH]):
            return m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9


def launch_for(t_bag):
    best = None
    for d in glob.glob(os.path.join(LOGDIR, "20*-*-*-*-*-*-*")):
        lg = os.path.join(d, "launch.log")
        if not os.path.exists(lg):
            continue
        txt = open(lg, errors="replace").read()
        m = re.search(r"^(\d+\.\d+) \[INFO\] \[slam_pose_bridge\.py-\d+\]: process started with pid \[(\d+)\]",
                      txt, re.M)
        if not m:
            continue
        t_start, pid = float(m.group(1)), int(m.group(2))
        if 0 <= t_bag - t_start <= 900 and (best is None or t_bag - t_start < best[0]):
            died = re.search(rf"^(\d+\.\d+) \[\w+\] \[slam_pose_bridge\.py-\d+\]: process has (?:died|finished)"
                             rf".*?pid {pid}", txt, re.M)
            best = (t_bag - t_start, d, pid, t_start, float(died.group(1)) if died else None)
    return best


def count(lines, t_lo, t_hi):
    out = {k: 0 for k, _ in CLASSES}
    episodes, last_streak = 0, None
    for t, ln in lines:
        if not (t_lo <= t <= t_hi):
            continue
        for k, s in CLASSES:
            if s in ln:
                out[k] += 1
                break
        if "SLAM pose rejected" in ln:
            n = int(STREAK.search(ln).group(1))
            if last_streak is None or n <= last_streak:
                episodes += 1
            last_streak = n
        elif any(s in ln for s in ("SLAM alignment solved", "SLAM alignment stale", "Map point crash",
                                   "SLAM jump detected", "SLAM QUARANTINED")):
            last_streak = None
    out["reject_episodes"] = episodes
    return out


def main():
    scored = {r["bag"]: r for r in json.load(open(ps.OUT_JSON))["flights"]}
    res = []
    for f in ps.flights():
        bag = os.path.join(ROOT, "bags", f["bag"])
        t0 = first_truth_time(bag)
        t_bag, t_stop = ps.bag_span(bag)
        s = scored[f["bag"]]
        cut, end = t0 + s["cutoff"], t0 + s["t_complete"]
        L = launch_for(t_bag)
        row = dict(arm=f["arm"], seed=f["seed"], bag=f["bag"])
        if L is None:
            res.append(dict(row, covered=False, why="no launch with a bridge within 900 s before the bag"))
            continue
        _, d, pid, t_start, t_died = L
        logs = glob.glob(os.path.join(LOGDIR, f"python3_{pid}_*.log"))
        if len(logs) != 1:
            res.append(dict(row, covered=False, why=f"{len(logs)} bridge logs for pid {pid}"))
            continue
        lines = []
        for ln in open(logs[0], errors="replace"):
            m = TS.search(ln)
            if m:
                lines.append((float(m.group(1)), ln))
        head = "".join(ln for _, ln in lines[:3])
        ref = re.search(r"reference_odom=(\S+?),", head)
        alive = t_start < t_bag and (t_died is None or t_died > end)
        inside = any(cut - 600 <= t <= end for t, _ in lines)
        covered = bool(alive and inside and ref and ref.group(1) == "/bluerov2/odometry/dr")
        res.append(dict(row, covered=covered, launch=os.path.basename(d.rstrip("/")),
                        bridge_log=os.path.basename(logs[0]), reference_odom=ref.group(1) if ref else None,
                        alive_whole_mission=bool(alive), died=t_died is not None,
                        before_cutoff=count(lines, t0, cut), to_end=count(lines, t0, end)))
    json.dump(res, open(OUT_JSON, "w"), indent=1)

    keys = ["lock", "refit", "reject_episodes", "gate_reject", "stale_realign", "quarantine",
            "jump_reset", "jump_new_map", "map_crash", "fit_rejected", "refit_rejected", "probation"]
    md = ["# Pre-registered G5: bridge events per mission", "",
          "Generated by `eval/bridge_events.py`; do not edit by hand. Counts before the primary "
          "cutoff (RETURN_HOME start); in brackets, to mission end.", "",
          "| arm | seed | covered | " + " | ".join(keys) + " |",
          "|---|---|---|" + "---|" * len(keys)]
    for r in sorted(res, key=lambda r: (ps.ARMS.index(r["arm"]), r["seed"])):
        if not r["covered"]:
            md.append(f"| {r['arm']} | {r['seed']} | NO: {r.get('why', r.get('reference_odom'))} |" + " |" * len(keys))
            continue
        b, e = r["before_cutoff"], r["to_end"]
        md.append(f"| {r['arm']} | {r['seed']} | yes | " +
                  " | ".join(f"{b[k]} ({e[k]})" for k in keys) + " |")
    md += ["", "Per arm, missions with at least one event before the cutoff (to mission end):", ""]
    for a in ps.ARMS:
        R = [r for r in res if r["arm"] == a and r["covered"]]
        md.append(f"- {a} ({len(R)} covered): " + ", ".join(
            f"{k} {sum(r['before_cutoff'][k] > 0 for r in R)} ({sum(r['to_end'][k] > 0 for r in R)})"
            for k in ("reject_episodes", "stale_realign", "quarantine", "jump_reset", "map_crash")))
    open(OUT_MD, "w").write("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
