#!/usr/bin/env python3
"""Was the vehicle really somewhere it had been before, at each closure?

A closure is CORRECT only if the place it matches was visited earlier. The
classifier in closure_events.py judges closures by the geometry of the
correction; this judges them by the ground-truth trajectory, which is an
independent line of evidence.

Two fixed-in-advance checks (NEAR = 5 m, LAG = 120 s):

  P1  at a face-aliased closure the vehicle's TRUE position has no visit
      within NEAR in its own track older than LAG; at a benign closure it
      does.
  P2  at a face-aliased closure the estimate AFTER the closure lands within
      NEAR of a place the vehicle really did visit earlier -- the match is to
      a real place, just not this one.

Usage:  closure_revisit.py BAGNAME [BAGNAME ...]   or   closure_revisit.py FILE
        (FILE: one bag name per line; bags found under bags/ or the media
        drive.) Ground truth is used to score only.
"""
import os, sys, glob, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import closure_events as ce
from mcap_ros2.reader import read_ros2_messages

LAG, NEAR = 120.0, 5.0     # fixed before computing

def truth3(bag):
    r=[]
    for f in sorted(glob.glob(os.path.join(bag,"*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/odometry"]):
            t=(m.log_time.timestamp() if hasattr(m.log_time,"timestamp") else m.log_time/1e9)
            p=m.ros_msg.pose.pose.position; r.append((t,p.x,p.y,p.z))
    return np.array(r)

names=([l.strip() for l in open(sys.argv[1]) if l.strip()]
       if len(sys.argv)==2 and os.path.isfile(sys.argv[1]) else sys.argv[1:])
print(f"{'mission':26s} {'t(s)':>5s} {'class':13s} {'true pos: nearest EARLIER visit':>34s} {'estimate AFTER: nearest true visit':>36s}")
res=[]
for b in names:
    p="/media/yadunandan/more space/bags/"+b
    if not os.path.isdir(p): p="bags/"+b
    log=ce.pick_log(p)
    rows,_=ce.classify(p, log)
    tr=truth3(p); t0=tr[0,0]; rel=tr[:,0]-t0
    for r in rows:
        if r["cls"]=="unscorable": continue
        te=r["t"]; i=int(np.argmin(abs(rel-te)))
        pos=tr[i,1:3]
        old=rel<=te-LAG
        d=np.hypot(tr[old,1]-pos[0], tr[old,2]-pos[1])
        j=int(np.argmin(d)); dmin=d[j]; when=rel[old][j]
        s1=f"{dmin:6.1f} m (at t={when:4.0f}s)"
        if r["cls"] in ("face-aliased","other-large"):
            ra=r["r_before"]*(1+r["dr"]); ba=math.radians(r["bearing_after"])
            aft=np.array([ce.TURBINE[0]+ra*math.cos(ba), ce.TURBINE[1]+ra*math.sin(ba)])
            d2=np.hypot(tr[old,1]-aft[0], tr[old,2]-aft[1]); k=int(np.argmin(d2))
            s2=f"{d2[k]:6.1f} m (at t={rel[old][k]:4.0f}s)"; dafter=d2[k]
        else:
            s2="            n/a"; dafter=None
        print(f"{b[-26:]:26s} {te:5.0f} {r['cls']:13s} {s1:>34s} {s2:>36s}")
        res.append((r["cls"],dmin,dafter))
print()
for cls in ("benign","face-aliased","other-large"):
    v=[d for c,d,_ in res if c==cls]
    if v: print(f"{cls:13s} n={len(v)}  nearest earlier visit to TRUE position: min {min(v):.1f}  max {max(v):.1f} m")
al=[a for c,_,a in res if c=="face-aliased"]
if al:
    print(f"face-aliased  estimate-after to nearest true earlier visit: max {max(al):.1f} m")
# A prediction about a class with no members was not tested; say so rather
# than reporting a vacuous HOLDS.
nb = sum(1 for c,_,_ in res if c=="benign")
ok1 = all(d>NEAR for c,d,_ in res if c=="face-aliased") and all(d<=NEAR for c,d,_ in res if c=="benign")
ok2 = all(a<=NEAR for a in al)
tested1 = bool(al) or bool(nb)
print(f"\nP1 (aliased: no earlier visit >{NEAR:.0f} m;  benign: one within {NEAR:.0f} m): "
      f"{('HOLDS' if ok1 else 'FAILS') if tested1 else 'NOT TESTED'}  (aliased n={len(al)}, benign n={nb})")
print(f"P2 (aliased estimate lands within {NEAR:.0f} m of a true earlier visit):        "
      f"{('HOLDS' if ok2 else 'FAILS') if al else 'NOT TESTED (no face-aliased closure)'}")
