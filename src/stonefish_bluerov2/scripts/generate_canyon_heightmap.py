#!/usr/bin/env python3
"""
Task G — procedural canyon heightmap for Stonefish terrain-following work.

A navigable canyon corridor: narrow flat floor channel running the full
length of the Y axis (open at both ends -- something to swim *through*,
not a closed bowl), STEEP walls close on either side, textured with mild
Perlin roughness. Beyond the rim, terrain flattens back into a normal
seabed plateau, matching how a real canyon/gorge actually reads visually
(steep walls framing the view, ordinary seafloor beyond them) rather than
continuing to climb all the way to the map edge.

Saved as a 16-bit grayscale PNG for Stonefish's Terrain height_map
loader (entities/statics/Terrain.cpp: type="terrain").

Three bugs fixed across iterations, each confirmed against Terrain.cpp's
actual loader code and/or a live in-sim render, not assumption:

1. Stonefish inverts pixel value to elevation:
   `elevation = (1 - pixel/65535) * height` -- HIGH pixel value = LOW
   elevation, opposite of the usual "white = high ground" intuition.
   The valley needs a HIGH pixel value at the floor and LOW at the
   walls' outer edge.
2. Noise amplitude was much larger than the valley carve's amplitude
   (~[-1,1] vs [0,0.6]), so the noise dominated the visible shape --
   rendered as chaotic spikes with no coherent channel. Noise is now
   scaled down to a texture role; the valley carve dominates the shape.
3. The first working version had a 43m-wide flat floor and let the
   walls rise gradually over the *entire remaining 55m* out to the map
   edge -- a 15m rise over that much horizontal run is only a ~15
   degree slope, which read as "normal seafloor" in an actual render,
   not a canyon. Walls now rise steeply over a short run right at the
   floor's edge, then plateau at full height -- narrow, steep-walled
   gorge instead of one long, gentle slope across the whole map.
"""
import os
import numpy as np
from noise import pnoise2
from PIL import Image

SIZE = 1024
SCALE = 0.15  # m/px, must match canyon_terrain.scn's dimensions scalex/scaley
NOISE_SCALE = 80.0
NOISE_AMPLITUDE = 0.15    # texture only, must stay well below the valley amplitude
VALLEY_AMPLITUDE = 1.0
FLOOR_HALF_WIDTH = 0.06   # ~9m-wide flat floor -- a narrow, intimate channel
WALL_HALF_WIDTH = 0.22    # wall rise completes by here; flat plateau beyond
WALL_STEEPNESS = 1.2      # >1 = slightly eased into the rise, still steep overall
OUT = "/home/yadunandan/ros2_ws/src/stonefish_bluerov2/data/terrain/canyon_heightmap.png"

noise = np.zeros((SIZE, SIZE), dtype=np.float64)
for y in range(SIZE):
    for x in range(SIZE):
        noise[y, x] = pnoise2(x / NOISE_SCALE, y / NOISE_SCALE,
                               octaves=6, persistence=0.55, lacunarity=2.2)

# Three-zone wall profile, constant along y (open corridor, not a closed
# pit): flat floor -> steep rise -> flat rim plateau.
x = np.linspace(-1, 1, SIZE)
ax = np.abs(x)
wall = np.clip((ax - FLOOR_HALF_WIDTH) / (WALL_HALF_WIDTH - FLOOR_HALF_WIDTH), 0, 1)
wall = wall ** WALL_STEEPNESS
valley = 1.0 - wall  # 1.0 on the floor, 0.0 from the rim outward (plateau)

heightmap = valley[None, :] * VALLEY_AMPLITUDE + noise * NOISE_AMPLITUDE
heightmap = (heightmap - heightmap.min()) / (heightmap.max() - heightmap.min())

os.makedirs(os.path.dirname(OUT), exist_ok=True)
Image.fromarray((heightmap * 65535).astype(np.uint16)).save(OUT)
print(f"saved {OUT}  ({SIZE}x{SIZE}, 16-bit)")
print(f"value range check: min={heightmap.min():.3f} max={heightmap.max():.3f}")

half_m = SIZE * SCALE / 2.0
floor_w_m = 2 * FLOOR_HALF_WIDTH * half_m
wall_run_m = (WALL_HALF_WIDTH - FLOOR_HALF_WIDTH) * half_m
print(f"floor width: {floor_w_m:.1f} m | wall horizontal run: {wall_run_m:.1f} m "
      f"(steep rise over that distance, then flat rim plateau)")

mid = SIZE // 2
print("center row pixel values (x=0,25%,50%,75%,100%):",
      (heightmap[mid, [0, SIZE//4, SIZE//2, 3*SIZE//4, SIZE-1]] * 65535).astype(int))
