#!/usr/bin/env python3
"""
Task G — procedural canyon heightmap for Stonefish terrain-following work.

A navigable canyon corridor: flat(ish) floor channel running the full
length of the Y axis (open at both ends -- something to swim *through*,
not a closed bowl), walls rising only in X, textured with mild Perlin
roughness.

Saved as a 16-bit grayscale PNG for Stonefish's Terrain height_map
loader (entities/statics/Terrain.cpp: type="terrain").

Two bugs fixed from the first pass, both confirmed against
Terrain.cpp's actual loader code and a live render, not assumption:

1. Stonefish inverts pixel value to elevation:
   `elevation = (1 - pixel/65535) * height` -- HIGH pixel value = LOW
   elevation, opposite of the usual "white = high ground" intuition.
   The valley needs a HIGH pixel value at the floor and LOW at the
   walls' outer edge.
2. Noise amplitude was much larger than the valley carve's amplitude
   (~[-1,1] vs [0,0.6]), so the noise dominated the visible shape --
   rendered as chaotic spikes with no coherent channel, confirmed by
   an actual in-sim screenshot. Noise is now scaled down to a texture
   role; the valley carve dominates the shape.
"""
import os
import numpy as np
from noise import pnoise2
from PIL import Image

SIZE = 1024
NOISE_SCALE = 80.0
NOISE_AMPLITUDE = 0.15   # texture only, must stay well below the valley amplitude
VALLEY_AMPLITUDE = 1.0
FLOOR_HALF_WIDTH = 0.28  # fraction of half-width that stays flat channel floor
OUT = "/home/yadunandan/ros2_ws/src/stonefish_bluerov2/data/terrain/canyon_heightmap.png"

noise = np.zeros((SIZE, SIZE), dtype=np.float64)
for y in range(SIZE):
    for x in range(SIZE):
        noise[y, x] = pnoise2(x / NOISE_SCALE, y / NOISE_SCALE,
                               octaves=6, persistence=0.55, lacunarity=2.2)

# Wall profile: flat floor for |x| < FLOOR_HALF_WIDTH, then a smooth
# rise to the canyon rim at |x| = 1. Constant along y -- an open
# corridor, not a closed pit.
x = np.linspace(-1, 1, SIZE)
wall = np.clip((np.abs(x) - FLOOR_HALF_WIDTH) / (1 - FLOOR_HALF_WIDTH), 0, 1)
wall = wall ** 1.5
valley = 1.0 - wall  # 1.0 on the floor, falling to 0.0 at the rim

heightmap = valley[None, :] * VALLEY_AMPLITUDE + noise * NOISE_AMPLITUDE
heightmap = (heightmap - heightmap.min()) / (heightmap.max() - heightmap.min())

os.makedirs(os.path.dirname(OUT), exist_ok=True)
Image.fromarray((heightmap * 65535).astype(np.uint16)).save(OUT)
print(f"saved {OUT}  ({SIZE}x{SIZE}, 16-bit)")
print(f"value range check: min={heightmap.min():.3f} max={heightmap.max():.3f}")

mid = SIZE // 2
print("center row pixel values (x=0,25%,50%,75%,100%):",
      (heightmap[mid, [0, SIZE//4, SIZE//2, 3*SIZE//4, SIZE-1]] * 65535).astype(int))
