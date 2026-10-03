#!/usr/bin/env python3
"""
Publication figure style for the paper.

The existing figures are diagnostic plots -- fine for deciding what happened,
wrong for a manuscript. Journal figures have constraints screen plots do not:

  VECTOR OUTPUT. Raster figures look soft in print and reviewers notice.
  Everything is written as PDF for LaTeX, with a PNG alongside for previewing.

  REAL COLUMN WIDTH. A figure authored at arbitrary size and then scaled into a
  column has text at the wrong size relative to the caption. Authoring at the
  true column width means 8 pt in the figure is 8 pt on the page.

  GRAYSCALE SURVIVAL. Readers print in black and white. The palette below passes
  CVD separation (checked: normal >=24, deuteranopia >=9.9, protanopia >=13.4 in
  OKLab x100) but its grayscale separation is only 0.045-0.135 in luminance, so
  colour alone cannot carry identity. Every series therefore also gets a distinct
  marker and linestyle, which is the same rule the accessibility pass asks for.

  RECESSIVE FURNITURE. Grid and axes are support, not data: thin, light, behind.
  Top and right spines removed. No boxed-and-shadowed legends.

Palette is the reference categorical set, slots taken in fixed order and not
recoloured per chart, so a series keeps its colour across every figure in the
paper.

Usage:
    from pubstyle import use, save, COL1, COL2, SERIES, MARKERS
    use()
    fig, ax = plt.subplots(figsize=(COL1, COL1 * 0.72))
    ...
    save(fig, 'eval/figures/task10_alloc')     # writes .pdf and .png
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Journal column widths in inches. Most Elsevier/MDPI single columns sit near
# 3.5 in; double near 7.16 in. Author at these and do not rescale afterwards.
COL1, COL2 = 3.46, 7.16

# Reference categorical palette, fixed order. Do not cycle or reassign per chart.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300']
# Paired with the colours so identity survives grayscale printing.
MARKERS = ['o', 's', '^', 'D', 'v', 'P']
LINESTYLES = ['-', '--', '-.', ':', (0, (3, 1, 1, 1)), (0, (5, 1))]

INK = '#1a1a19'
INK_SECONDARY = '#52514e'
GRID = '#d8d7d2'


def use(base_fontsize=8):
    """Apply the publication rcParams. Call once before plotting."""
    plt.rcParams.update({
        'figure.dpi': 150,
        'savefig.dpi': 600,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,
        'pdf.fonttype': 42,          # embed TrueType, not Type 3 -- many
        'ps.fonttype': 42,           # journals reject Type 3 outright
        'font.family': 'sans-serif',
        'font.size': base_fontsize,
        'axes.titlesize': base_fontsize,
        'axes.labelsize': base_fontsize,
        'xtick.labelsize': base_fontsize - 1,
        'ytick.labelsize': base_fontsize - 1,
        'legend.fontsize': base_fontsize - 1,
        'axes.edgecolor': INK_SECONDARY,
        'axes.labelcolor': INK,
        'text.color': INK,
        'xtick.color': INK_SECONDARY,
        'ytick.color': INK_SECONDARY,
        'axes.linewidth': 0.6,
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.major.size': 2.5,
        'ytick.major.size': 2.5,
        'lines.linewidth': 1.2,
        'lines.markersize': 3.5,
        'axes.grid': True,
        'axes.grid.axis': 'y',
        'grid.color': GRID,
        'grid.linewidth': 0.5,
        'axes.axisbelow': True,       # grid behind the data, never over it
        'legend.frameon': False,
        'legend.handlelength': 1.6,
        'legend.columnspacing': 1.0,
        'legend.borderaxespad': 0.3,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
    })


def despine(ax, keep=('left', 'bottom')):
    for side in ('top', 'right', 'left', 'bottom'):
        ax.spines[side].set_visible(side in keep)


def save(fig, stem, also_png=True):
    """Write <stem>.pdf (for LaTeX) and <stem>.png (for previewing)."""
    os.makedirs(os.path.dirname(stem) or '.', exist_ok=True)
    out = [f'{stem}.pdf']
    fig.savefig(out[0])
    if also_png:
        fig.savefig(f'{stem}.png')
        out.append(f'{stem}.png')
    plt.close(fig)
    return out
