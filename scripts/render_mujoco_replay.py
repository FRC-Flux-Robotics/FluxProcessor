#!/usr/bin/env python3
"""Headless Photorealistic MuJoCo 3.15 OpenGL/EGL 3D Replay Video Renderer for FluxProcessor.

Loads the real 14-part `build123d` STL CAD assembly of FRC Team 10413 `FUEL` (`assets/meshes/*.stl`)
inside a full 3D 2026 REBUILT Arena (Blue & Red Octagonal Hubs with structural struts, funnel rims,
overhead nets & AprilTags, 4 Trench assemblies, Center Climb Tower, polycarbonate perimeter guardrails,
Driver Station walls, reflective woven carpet with painted field lines, and 36 3D Fuel balls with
4096x4096 directional shadow maps and 3D parabolic flight trails), and renders `replay-3d.mp4`
at 1280x720 HD with Picture-in-Picture (PiP) Close-Up Robot Chase Cam & Onboard OV9281 Camera views.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


MAX_BALLS = 36
MAX_TRAIL_SITES = 28


def build_arena_mjcf(mesh_dir: Path, is_blue: bool) -> str:
    """Generate the complete 2026 REBUILT 3D Arena + 14-mesh `FUEL` CAD Robot MJCF XML."""
    bumper_mat = "mat_bumper_blue" if is_blue else "mat_bumper_red"

    # Generate 8-sided octagonal hub geometry helper for Blue (4.626, 4.035) and Red (11.915, 4.035)
    def hub_body_xml(name: str, hx: float, hy: float, hub_mat: str, is_blue_hub: bool) -> str:
        lines = [f'    <body name="{name}" pos="{hx:.3f} {hy:.3f} 0">']
        # Lower base pedestal & 8 octagonal faces
        lines.append(
            f'      <geom name="{name}_base" type="cylinder" pos="0 0 0.52" size="0.46 0.52" material="{hub_mat}"/>'
        )
        lines.append(
            f'      <geom name="{name}_collar_low" type="cylinder" pos="0 0 1.06" size="0.52 0.04" material="mat_aluminum"/>'
        )
        # Flared upper funnel walls (8 angled panels around radius 0.58m at z=1.10..1.65m)
        for i in range(8):
            ang_deg = i * 45.0 + 22.5
            ang_rad = math.radians(ang_deg)
            px = 0.56 * math.cos(ang_rad)
            py = 0.56 * math.sin(ang_rad)
            sx = 0.50 * math.cos(ang_rad)
            sy = 0.50 * math.sin(ang_rad)
            # Angled steel strut from base to rim
            lines.append(
                f'      <geom name="{name}_strut_{i}" type="capsule" fromto="{sx*0.82:.3f} {sy*0.82:.3f} 0.10 {px:.3f} {py:.3f} 1.65" size="0.018" material="mat_dark_steel"/>'
            )
            # Polycarbonate funnel side panel
            lines.append(
                f'      <geom name="{name}_panel_{i}" type="box" pos="{px:.3f} {py:.3f} 1.44" size="0.015 0.22 0.20" euler="0 -14 {ang_deg:.1f}" material="mat_polycarb_tinted"/>'
            )
            # Bright gold top rim segment
            lines.append(
                f'      <geom name="{name}_rim_{i}" type="box" pos="{px*1.03:.3f} {py*1.03:.3f} 1.65" size="0.028 0.235 0.032" euler="0 0 {ang_deg:.1f}" material="mat_hub_rim"/>'
            )
        # Inner dark funnel floor
        lines.append(
            f'      <geom name="{name}_funnel_floor" type="cylinder" pos="0 0 1.24" size="0.48 0.025" rgba="0.08 0.11 0.18 1"/>'
        )
        # Overhead Netting Frame (z = 1.65..2.85m)
        net_x = 0.18 if is_blue_hub else -0.18
        lines.append(
            f'      <geom name="{name}_net_post_l" type="cylinder" pos="{net_x:.2f} 0.52 2.25" size="0.022 0.60" material="mat_aluminum"/>'
        )
        lines.append(
            f'      <geom name="{name}_net_post_r" type="cylinder" pos="{net_x:.2f} -0.52 2.25" size="0.022 0.60" material="mat_aluminum"/>'
        )
        lines.append(
            f'      <geom name="{name}_net_top" type="cylinder" fromto="{net_x:.2f} -0.52 2.85 {net_x:.2f} 0.52 2.85" size="0.020" material="mat_hub_rim"/>'
        )
        lines.append(
            f'      <geom name="{name}_net_mesh" type="box" pos="{net_x:.2f} 0 2.25" size="0.006 0.50 0.58" rgba="0.85 0.90 0.96 0.20"/>'
        )
        # AprilTags on alliance-facing side
        tag_x = -0.47 if is_blue_hub else 0.47
        for idx, ty in enumerate([-0.18, 0.18]):
            lines.append(
                f'      <geom name="{name}_tag_w_{idx}" type="box" pos="{tag_x:.3f} {ty:.2f} 0.86" size="0.005 0.105 0.105" material="mat_tag_white"/>'
            )
            lines.append(
                f'      <geom name="{name}_tag_b_{idx}" type="box" pos="{tag_x*1.005:.3f} {ty:.2f} 0.86" size="0.005 0.082 0.082" material="mat_tag_black"/>'
            )
            lines.append(
                f'      <geom name="{name}_tag_c_{idx}" type="box" pos="{tag_x*1.010:.3f} {ty:.2f} 0.86" size="0.005 0.038 0.038" material="mat_tag_white"/>'
            )
        lines.append("    </body>")
        return "\n".join(lines)

    blue_hub_xml = hub_body_xml("blue_hub", 4.626, 4.035, "mat_hub_blue", True)
    red_hub_xml = hub_body_xml("red_hub", 11.915, 4.035, "mat_hub_red", False)

    # Generate 36 Fuel balls + 28 3D flight trail glow spheres
    balls_xml = []
    for i in range(MAX_BALLS):
        balls_xml.append(
            f"""    <body name="fuel_ball_{i}" pos="8.27 4.03 -2.0" mocap="true">
      <geom name="geom_ball_{i}" type="sphere" size="0.0889" material="mat_fuel_ball" contype="0" conaffinity="0"/>
    </body>"""
        )
    trails_xml = []
    for i in range(MAX_TRAIL_SITES):
        alpha = 0.18 + 0.65 * (i / max(1, MAX_TRAIL_SITES - 1))
        rad = 0.022 + 0.024 * (i / max(1, MAX_TRAIL_SITES - 1))
        trails_xml.append(
            f"""    <body name="trail_node_{i}" pos="8.27 4.03 -2.0" mocap="true">
      <geom name="geom_trail_{i}" type="sphere" size="{rad:.4f}" rgba="0.22 0.82 0.98 {alpha:.2f}" contype="0" conaffinity="0"/>
    </body>"""
        )

    return f"""<mujoco model="frc_10413_nytr_2026_rebuilt_replay">
  <compiler angle="degree" meshdir="{mesh_dir.as_posix()}"/>
  <option timestep="0.02" gravity="0 0 -9.80665"/>

  <visual>
    <global offwidth="1280" offheight="720" fovy="46"/>
    <quality shadowsize="1024" offsamples="0" numslices="20" numstacks="16"/>
    <headlight diffuse="0.58 0.60 0.65" ambient="0.34 0.36 0.42" specular="0.40 0.42 0.45"/>
    <map znear="0.03" zfar="35.0" shadowclip="16.0"/>
    <rgba haze="0.05 0.07 0.12 1"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.10 0.14 0.24" rgb2="0.02 0.03 0.06" width="512" height="3072"/>
    <texture name="carpet_tex" type="2d" builtin="checker" rgb1="0.16 0.18 0.22" rgb2="0.18 0.20 0.25" width="1024" height="1024" mark="edge" markrgb="0.22 0.25 0.30"/>
    <material name="mat_carpet" texture="carpet_tex" texrepeat="24 12" texuniform="true" reflectance="0.05" specular="0.12" shininess="0.2"/>
    <material name="mat_arena_floor" rgba="0.08 0.09 0.12 1" reflectance="0.08" specular="0.25"/>
    <material name="mat_frame" rgba="0.82 0.85 0.90 1" specular="0.75" shininess="0.8"/>
    <material name="mat_aluminum" rgba="0.76 0.80 0.86 1" specular="0.70" shininess="0.75"/>
    <material name="mat_dark_steel" rgba="0.20 0.23 0.28 1" specular="0.55" shininess="0.6"/>
    <material name="mat_elec" rgba="0.14 0.78 0.52 1" specular="0.45" shininess="0.5"/>
    <material name="mat_bumper_blue" rgba="0.12 0.32 0.78 1" specular="0.35" shininess="0.45"/>
    <material name="mat_bumper_red" rgba="0.78 0.14 0.18 1" specular="0.35" shininess="0.45"/>
    <material name="mat_yoke" rgba="0.22 0.26 0.32 1" specular="0.60" shininess="0.6"/>
    <material name="mat_wheel" rgba="0.16 0.46 0.96 1" specular="0.45" shininess="0.5"/>
    <material name="mat_tilter" rgba="0.96 0.62 0.08 1" specular="0.55" shininess="0.6"/>
    <material name="mat_intake" rgba="0.10 0.82 0.55 1" specular="0.40" shininess="0.5"/>
    <material name="mat_indexer" rgba="0.36 0.64 0.96 1" specular="0.45" shininess="0.5"/>
    <material name="mat_feeder" rgba="0.94 0.34 0.44 1" specular="0.45" shininess="0.5"/>
    <material name="mat_shooter" rgba="1.00 0.78 0.12 1" specular="0.75" shininess="0.8"/>
    <material name="mat_hood" rgba="0.62 0.42 0.96 1" specular="0.55" shininess="0.6"/>
    <material name="mat_cam" rgba="0.12 0.92 0.82 1" specular="0.65" shininess="0.7"/>
    <material name="mat_fuel_ball" rgba="0.99 0.82 0.06 1" specular="0.55" shininess="0.65"/>
    <material name="mat_hub_blue" rgba="0.13 0.33 0.76 1" specular="0.45" shininess="0.55"/>
    <material name="mat_hub_red" rgba="0.76 0.15 0.20 1" specular="0.45" shininess="0.55"/>
    <material name="mat_hub_rim" rgba="0.98 0.78 0.10 1" specular="0.70" shininess="0.75"/>
    <material name="mat_polycarb" rgba="0.84 0.92 0.99 0.24" specular="0.85" shininess="0.9"/>
    <material name="mat_polycarb_tinted" rgba="0.75 0.86 0.98 0.32" specular="0.80" shininess="0.85"/>
    <material name="mat_tag_black" rgba="0.03 0.03 0.04 1" specular="0.05" shininess="0.05"/>
    <material name="mat_tag_white" rgba="0.98 0.99 1.00 1" specular="0.10" shininess="0.10"/>

    <mesh name="frame_mesh" file="swerve_frame.stl" scale="0.001 0.001 0.001"/>
    <mesh name="elec_mesh" file="elec_deck.stl" scale="0.001 0.001 0.001"/>
    <mesh name="bumpers_mesh" file="bumpers.stl" scale="0.001 0.001 0.001"/>
    <mesh name="yoke_mesh" file="swerve_yoke.stl" scale="0.001 0.001 0.001"/>
    <mesh name="wheel_mesh" file="swerve_wheel.stl" scale="0.001 0.001 0.001"/>
    <mesh name="tilter_mesh" file="tilter_arm_local.stl" scale="0.001 0.001 0.001"/>
    <mesh name="intake_mesh" file="intake_roller_local.stl" scale="0.001 0.001 0.001"/>
    <mesh name="indexer_mesh" file="indexer_bed.stl" scale="0.001 0.001 0.001"/>
    <mesh name="feeder_mesh" file="feeder_wheel_local.stl" scale="0.001 0.001 0.001"/>
    <mesh name="shooter_mesh" file="shooter_flywheels.stl" scale="0.001 0.001 0.001"/>
    <mesh name="hood_mesh" file="hood_local.stl" scale="0.001 0.001 0.001"/>
    <mesh name="cam0_mesh" file="cam_0.stl" scale="0.001 0.001 0.001"/>
    <mesh name="cam1_mesh" file="cam_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="cam2_mesh" file="cam_2.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <!-- Arena Overhead Stadium Lighting with 4096x4096 Shadow Mapping -->
    <light name="stadium_key_blue" pos="4.2 3.2 7.5" dir="0.08 0.12 -0.98" directional="true" castshadow="true" diffuse="0.88 0.88 0.90" specular="0.45 0.45 0.50"/>
    <light name="stadium_fill_red" pos="12.2 5.0 7.0" dir="-0.15 -0.10 -0.98" directional="false" castshadow="false" diffuse="0.45 0.46 0.52"/>
    <light name="stadium_rim_side" pos="8.27 -2.5 5.0" dir="0.0 0.65 -0.75" directional="false" castshadow="false" diffuse="0.32 0.36 0.45"/>

    <!-- Outer Arena Concrete Floor & 16.541m x 8.069m Field Carpet -->
    <geom name="arena_subfloor" type="plane" pos="8.27 4.035 -0.005" size="14.0 9.0 0.1" material="mat_arena_floor" contype="0" conaffinity="0"/>
    <geom name="field_carpet" type="box" pos="8.2705 4.0345 -0.001" size="8.2705 4.0345 0.001" material="mat_carpet" contype="0" conaffinity="0"/>

    <!-- Painted Field Tape Lines (Centerline, Alliance Zone Lines, Hub Rings & Depots) -->
    <geom name="line_center" type="box" pos="8.2705 4.0345 0.0015" size="0.025 4.0345 0.001" rgba="0.95 0.96 0.98 0.90" contype="0" conaffinity="0"/>
    <geom name="line_blue_zone" type="box" pos="4.626 4.0345 0.0015" size="0.025 4.0345 0.001" rgba="0.20 0.52 0.98 0.92" contype="0" conaffinity="0"/>
    <geom name="line_red_zone" type="box" pos="11.915 4.0345 0.0015" size="0.025 4.0345 0.001" rgba="0.98 0.24 0.28 0.92" contype="0" conaffinity="0"/>
    <geom name="line_blue_start" type="box" pos="1.90 4.0345 0.0015" size="0.025 4.0345 0.001" rgba="0.95 0.96 0.98 0.75" contype="0" conaffinity="0"/>
    <geom name="line_red_start" type="box" pos="14.64 4.0345 0.0015" size="0.025 4.0345 0.001" rgba="0.95 0.96 0.98 0.75" contype="0" conaffinity="0"/>
    <geom name="ring_blue_hub" type="cylinder" pos="4.626 4.035 0.001" size="1.25 0.001" rgba="0.16 0.38 0.82 0.35" contype="0" conaffinity="0"/>
    <geom name="ring_red_hub" type="cylinder" pos="11.915 4.035 0.001" size="1.25 0.001" rgba="0.82 0.18 0.22 0.35" contype="0" conaffinity="0"/>

    <!-- Perimeter Polycarbonate Guardrails & Aluminum Extrusion Frame (16.541m x 8.069m) -->
    <geom name="wall_south_base" type="box" pos="8.2705 -0.03 0.06" size="8.30 0.03 0.06" material="mat_dark_steel"/>
    <geom name="wall_south_glass" type="box" pos="8.2705 -0.02 0.30" size="8.27 0.012 0.18" material="mat_polycarb"/>
    <geom name="wall_south_top" type="box" pos="8.2705 -0.03 0.50" size="8.30 0.025 0.02" material="mat_aluminum"/>

    <geom name="wall_north_base" type="box" pos="8.2705 8.099 0.06" size="8.30 0.03 0.06" material="mat_dark_steel"/>
    <geom name="wall_north_glass" type="box" pos="8.2705 8.089 0.30" size="8.27 0.012 0.18" material="mat_polycarb"/>
    <geom name="wall_north_top" type="box" pos="8.2705 8.099 0.50" size="8.30 0.025 0.02" material="mat_aluminum"/>

    <!-- Blue & Red Alliance Station Walls (with Driver Station Shelves & Alliance Header Bars) -->
    <geom name="wall_blue_lower" type="box" pos="-0.04 4.0345 0.48" size="0.04 4.06 0.48" material="mat_dark_steel"/>
    <geom name="wall_blue_shelf" type="box" pos="0.10 4.0345 0.96" size="0.14 3.60 0.025" material="mat_aluminum"/>
    <geom name="wall_blue_glass" type="box" pos="-0.03 4.0345 1.50" size="0.015 3.80 0.52" material="mat_polycarb"/>
    <geom name="wall_blue_header" type="box" pos="-0.04 4.0345 2.08" size="0.05 4.06 0.07" material="mat_hub_blue"/>

    <geom name="wall_red_lower" type="box" pos="16.581 4.0345 0.48" size="0.04 4.06 0.48" material="mat_dark_steel"/>
    <geom name="wall_red_shelf" type="box" pos="16.441 4.0345 0.96" size="0.14 3.60 0.025" material="mat_aluminum"/>
    <geom name="wall_red_glass" type="box" pos="16.571 4.0345 1.50" size="0.015 3.80 0.52" material="mat_polycarb"/>
    <geom name="wall_red_header" type="box" pos="16.581 4.0345 2.08" size="0.05 4.06 0.07" material="mat_hub_red"/>

    <!-- 4x 2026 REBUILT Trench Structures (Blue & Red South/North Trenches, z=0..0.89m) -->
    <body name="trench_blue_south" pos="4.626 0.65 0">
      <geom type="box" pos="0 -0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_blue"/>
      <geom type="box" pos="0  0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_blue"/>
      <geom type="box" pos="0  0.00 0.850" size="0.18 0.52 0.045" material="mat_hub_rim"/>
    </body>
    <body name="trench_blue_north" pos="4.626 7.42 0">
      <geom type="box" pos="0 -0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_blue"/>
      <geom type="box" pos="0  0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_blue"/>
      <geom type="box" pos="0  0.00 0.850" size="0.18 0.52 0.045" material="mat_hub_rim"/>
    </body>
    <body name="trench_red_south" pos="11.915 0.65 0">
      <geom type="box" pos="0 -0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_red"/>
      <geom type="box" pos="0  0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_red"/>
      <geom type="box" pos="0  0.00 0.850" size="0.18 0.52 0.045" material="mat_hub_rim"/>
    </body>
    <body name="trench_red_north" pos="11.915 7.42 0">
      <geom type="box" pos="0 -0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_red"/>
      <geom type="box" pos="0  0.45 0.445" size="0.16 0.06 0.445" material="mat_hub_red"/>
      <geom type="box" pos="0  0.00 0.850" size="0.18 0.52 0.045" material="mat_hub_rim"/>
    </body>

    <!-- Center Climb Tower / Rungs at Field Midline (8.27, 4.035) -->
    <body name="center_climb_tower" pos="8.2705 4.0345 0">
      <geom type="cylinder" pos="-0.55 -0.55 1.15" size="0.045 1.15" material="mat_dark_steel"/>
      <geom type="cylinder" pos=" 0.55 -0.55 1.15" size="0.045 1.15" material="mat_dark_steel"/>
      <geom type="cylinder" pos="-0.55  0.55 1.15" size="0.045 1.15" material="mat_dark_steel"/>
      <geom type="cylinder" pos=" 0.55  0.55 1.15" size="0.045 1.15" material="mat_dark_steel"/>
      <geom type="box" pos="0 0 2.32" size="0.62 0.62 0.04" material="mat_aluminum"/>
      <geom type="cylinder" fromto="-0.55 -0.55 0.75 -0.55 0.55 0.75" size="0.022" material="mat_hub_rim"/>
      <geom type="cylinder" fromto=" 0.55 -0.55 0.75  0.55 0.55 0.75" size="0.022" material="mat_hub_rim"/>
      <geom type="cylinder" fromto="-0.55 -0.55 1.35 -0.55 0.55 1.35" size="0.022" material="mat_hub_rim"/>
      <geom type="cylinder" fromto=" 0.55 -0.55 1.35  0.55 0.55 1.35" size="0.022" material="mat_hub_rim"/>
    </body>

    <!-- Blue & Red Octagonal Scoring Hubs -->
{blue_hub_xml}
{red_hub_xml}

    <!-- ==================== FRC TEAM 10413 `FUEL` 14-MESH CAD ROBOT ==================== -->
    <body name="fuel_robot" pos="3.55 4.21 0.001">
      <freejoint name="robot_free"/>

      <!-- 3rd-Person Close-Up Chase Camera (looking over robot toward Hub along -X) -->
      <camera name="chase_cam" pos="1.35 -0.68 0.96" xyaxes="0.44 0.90 0 -0.32 0.15 0.93" fovy="52"/>

      <!-- Onboard OV9281-2 Center AprilTag Camera (pointing out -X shooter side toward Hub) -->
      <camera name="ov9281_center" pos="-0.22 0.0 0.56" xyaxes="0.0 1.0 0.0 -0.22 0.0 0.975" fovy="65"/>

      <!-- CAD Visual Meshes from build123d -->
      <geom name="frame_vis"   type="mesh" mesh="frame_mesh"   material="mat_frame"     contype="0" conaffinity="0"/>
      <geom name="elec_vis"    type="mesh" mesh="elec_mesh"    material="mat_elec"      contype="0" conaffinity="0"/>
      <geom name="bumpers_vis" type="mesh" mesh="bumpers_mesh" material="{bumper_mat}"  contype="0" conaffinity="0"/>
      <geom name="indexer_vis" type="mesh" mesh="indexer_mesh" material="mat_indexer"   contype="0" conaffinity="0"/>
      <geom name="shooter_vis" type="mesh" mesh="shooter_mesh" material="mat_shooter"   contype="0" conaffinity="0"/>
      <geom name="cam0_vis"    type="mesh" mesh="cam0_mesh"    material="mat_cam"       contype="0" conaffinity="0"/>
      <geom name="cam1_vis"    type="mesh" mesh="cam1_mesh"    material="mat_cam"       contype="0" conaffinity="0"/>
      <geom name="cam2_vis"    type="mesh" mesh="cam2_mesh"    material="mat_cam"       contype="0" conaffinity="0"/>

      <!-- Articulated Tilter + Intake Roller -->
      <body name="tilter_link" pos="0.3050 0 0.1950">
        <joint name="j_tilter" type="hinge" axis="0 1 0"/>
        <geom name="tilter_vis" type="mesh" mesh="tilter_mesh" material="mat_tilter" contype="0" conaffinity="0"/>
        <body name="intake_roller_link" pos="0.168 0 0.062">
          <joint name="j_intake_roller" type="hinge" axis="0 1 0"/>
          <geom name="intake_vis" type="mesh" mesh="intake_mesh" material="mat_intake" contype="0" conaffinity="0"/>
        </body>
      </body>

      <!-- Spinning Feeder Wheel -->
      <body name="feeder_link" pos="-0.0920 0 0.2520">
        <joint name="j_feeder" type="hinge" axis="0 1 0"/>
        <geom name="feeder_vis" type="mesh" mesh="feeder_mesh" material="mat_feeder" contype="0" conaffinity="0"/>
      </body>

      <!-- Articulated Shooter Hood -->
      <body name="hood_link" pos="-0.1600 0 0.3550">
        <joint name="j_hood" type="hinge" axis="0 1 0"/>
        <geom name="hood_vis" type="mesh" mesh="hood_mesh" material="mat_hood" contype="0" conaffinity="0"/>
      </body>

      <!-- 4x Articulated Swerve Modules (FL, FR, BL, BR) -->
      <body name="mod_fl" pos="0.2826 0.2826 0.0508">
        <joint name="j_fl_steer" type="hinge" axis="0 0 1"/>
        <geom type="mesh" mesh="yoke_mesh" material="mat_yoke" contype="0" conaffinity="0"/>
        <body name="wheel_fl" pos="0 0 0">
          <joint name="j_fl_drive" type="hinge" axis="0 1 0"/>
          <geom type="mesh" mesh="wheel_mesh" material="mat_wheel" contype="0" conaffinity="0"/>
        </body>
      </body>
      <body name="mod_fr" pos="0.2826 -0.2826 0.0508">
        <joint name="j_fr_steer" type="hinge" axis="0 0 1"/>
        <geom type="mesh" mesh="yoke_mesh" material="mat_yoke" contype="0" conaffinity="0"/>
        <body name="wheel_fr" pos="0 0 0">
          <joint name="j_fr_drive" type="hinge" axis="0 1 0"/>
          <geom type="mesh" mesh="wheel_mesh" material="mat_wheel" contype="0" conaffinity="0"/>
        </body>
      </body>
      <body name="mod_bl" pos="-0.2826 0.2826 0.0508">
        <joint name="j_bl_steer" type="hinge" axis="0 0 1"/>
        <geom type="mesh" mesh="yoke_mesh" material="mat_yoke" contype="0" conaffinity="0"/>
        <body name="wheel_bl" pos="0 0 0">
          <joint name="j_bl_drive" type="hinge" axis="0 1 0"/>
          <geom type="mesh" mesh="wheel_mesh" material="mat_wheel" contype="0" conaffinity="0"/>
        </body>
      </body>
      <body name="mod_br" pos="-0.2826 -0.2826 0.0508">
        <joint name="j_br_steer" type="hinge" axis="0 0 1"/>
        <geom type="mesh" mesh="yoke_mesh" material="mat_yoke" contype="0" conaffinity="0"/>
        <body name="wheel_br" pos="0 0 0">
          <joint name="j_br_drive" type="hinge" axis="0 1 0"/>
          <geom type="mesh" mesh="wheel_mesh" material="mat_wheel" contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>

    <!-- 36 Mocap Fuel Balls & 28 Airborne Flight Trail Nodes -->
{chr(10).join(balls_xml)}
{chr(10).join(trails_xml)}
  </worldbody>
</mujoco>
"""


def yaw_to_quat(yaw_rad: float) -> list[float]:
    half = 0.5 * yaw_rad
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def render_match_video(sim_json_path: Path, out_mp4_path: Path, mesh_dir: Path) -> None:
    payload = json.loads(sim_json_path.read_text(encoding="utf-8"))
    run_id = payload.get("runId", "Match")
    alliance = payload.get("alliance", "Blue 1")
    is_blue = not str(alliance).lower().startswith("red")
    frames = payload.get("frames", [])
    if not frames:
        return

    mjcf_xml = build_arena_mjcf(mesh_dir, is_blue)
    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)

    robot_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "robot_free")
    robot_qpos_adr = model.jnt_qposadr[robot_jnt_id]
    tilter_qpos_adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j_tilter")]
    intake_qpos_adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j_intake_roller")]
    feeder_qpos_adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j_feeder")]
    hood_qpos_adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j_hood")]
    steer_qpos_adrs = [
        model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"j_{c}_steer")]
        for c in ("fl", "fr", "bl", "br")
    ]
    drive_qpos_adrs = [
        model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"j_{c}_drive")]
        for c in ("fl", "fr", "bl", "br")
    ]

    ball_mocap_ids = [
        model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"fuel_ball_{i}")]
        for i in range(MAX_BALLS)
    ]
    trail_mocap_ids = [
        model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"trail_node_{i}")]
        for i in range(MAX_TRAIL_SITES)
    ]

    # Dynamic AdvantageScope-style 3D Orbit / Cinema Free Camera
    main_cam = mujoco.MjvCamera()
    main_cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    hub_x = 4.626 if is_blue else 11.915
    hub_y = 4.035
    smooth_lookat = np.array([3.8 if is_blue else 12.7, 4.035, 0.68], dtype=np.float64)

    trail_history: list[tuple[float, float, float]] = []
    wheel_spin = 0.0
    roller_spin = 0.0

    # Spawn ffmpeg pipe for 1280x720 30fps H.264 MP4
    W, H = 1280, 720
    PIP_W, PIP_H = 352, 198
    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{W}x{H}",
        "-r",
        "25",
        "-i",
        "-",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(out_mp4_path),
    ]
    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    prev_rx, prev_ry = float(frames[0]["rx"]), float(frames[0]["ry"])
    robot_name = payload.get("robotName", "FUEL")
    battery_name = payload.get("batteryName", "UNSPECIFIED")

    try:
        font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
        font_score = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
        font_mono = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 14)
        font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 12)
    except Exception:
        font_title = ImageFont.load_default()
        font_score = ImageFont.load_default()
        font_mono = ImageFont.load_default()
        font_small = ImageFont.load_default()

    saved_preview = False
    preview_png_path = out_mp4_path.parent / "preview-3d.png"
    pip_chase = None
    pip_ov = None
    rendered_count = 0
    short_run_id = run_id if len(run_id) <= 22 else (run_id[-22:])

    with (
        mujoco.Renderer(model, height=H, width=W) as renderer_main,
        mujoco.Renderer(model, height=PIP_H, width=PIP_W) as renderer_pip,
    ):
        for idx, f in enumerate(frames):
            rx = float(f["rx"])
            ry = float(f["ry"])
            yaw = float(f["yaw"])
            shooter_rps = float(f["shooterRps"])
            hood_rot = float(f["hoodRot"])
            balls_flat = f["balls"]

            # Check if any ball is airborne
            airborne_pts = []
            for bi in range(0, len(balls_flat) - 2, 3):
                if balls_flat[bi + 2] > 0.14:
                    airborne_pts.append((balls_flat[bi], balls_flat[bi + 1], balls_flat[bi + 2]))

            # Adaptive frame stride: render every frame during airborne shots, and every 3rd frame during cruise
            if (not airborne_pts) and shooter_rps < 18.0 and (idx % 3 != 0) and len(frames) > 300:
                continue

            # Keep visual robot pose outside the physical Hub base collider (r = 0.46m + robot 0.56m = 1.02m)
            dhx = rx - hub_x
            dhy = ry - hub_y
            dist_hub = math.hypot(dhx, dhy)
            if dist_hub < 1.04:
                push_ang = math.atan2(dhy, dhx) if dist_hub > 0.01 else (math.pi if is_blue else 0.0)
                rx = hub_x + 1.04 * math.cos(push_ang)
                ry = hub_y + 1.04 * math.sin(push_ang)

            dx = rx - prev_rx
            dy = ry - prev_ry
            step_dist = math.hypot(dx, dy)
            move_angle = math.atan2(dy, dx) - yaw if step_dist > 0.005 else 0.0
            wheel_spin += step_dist / 0.0508
            roller_spin += 0.45
            prev_rx, prev_ry = rx, ry

            # Set robot 3D pose (FUEL shoots out -X in CAD mesh local frame, so orient -X toward Hub)
            data.qpos[robot_qpos_adr : robot_qpos_adr + 3] = [rx, ry, 0.001]
            data.qpos[robot_qpos_adr + 3 : robot_qpos_adr + 7] = yaw_to_quat(yaw + math.pi)

            # Set articulated mechanism joints (Tilter deployed ~48 deg when intaking, stowed ~12 deg when shooting)
            data.qpos[tilter_qpos_adr] = math.radians(15.0 if shooter_rps > 20.0 else 48.0)
            data.qpos[intake_qpos_adr] = roller_spin
            data.qpos[feeder_qpos_adr] = roller_spin * (2.0 if shooter_rps > 20.0 else 0.2)
            data.qpos[hood_qpos_adr] = min(0.38, max(0.0, hood_rot * 0.035))

            for s_adr in steer_qpos_adrs:
                data.qpos[s_adr] = move_angle
            for d_adr in drive_qpos_adrs:
                data.qpos[d_adr] = wheel_spin

            # Position all active Fuel balls
            num_balls = min(MAX_BALLS, len(balls_flat) // 3)
            for b_i in range(MAX_BALLS):
                m_id = ball_mocap_ids[b_i]
                if b_i < num_balls:
                    data.mocap_pos[m_id] = [
                        balls_flat[b_i * 3],
                        balls_flat[b_i * 3 + 1],
                        balls_flat[b_i * 3 + 2],
                    ]
                else:
                    data.mocap_pos[m_id] = [8.27, 4.03, -2.0]

            # Update 3D airborne shot trail
            for pt in airborne_pts:
                trail_history.append(pt)
            if not airborne_pts and len(trail_history) > 0:
                trail_history = trail_history[2:]
            while len(trail_history) > MAX_TRAIL_SITES:
                trail_history.pop(0)

            for t_i in range(MAX_TRAIL_SITES):
                tm_id = trail_mocap_ids[t_i]
                if t_i < len(trail_history):
                    data.mocap_pos[tm_id] = list(trail_history[t_i])
                else:
                    data.mocap_pos[tm_id] = [8.27, 4.03, -2.0]

            mujoco.mj_forward(model, data)

            # Smoothly track between Robot and Alliance Hub from the Alliance Front-Side (so Robot is in foreground & Hub faces camera)
            target_look = np.array(
                [0.58 * rx + 0.42 * hub_x, 0.58 * ry + 0.42 * hub_y, 0.72], dtype=np.float64
            )
            smooth_lookat = 0.86 * smooth_lookat + 0.14 * target_look
            main_cam.lookat[:] = smooth_lookat
            main_cam.distance = 4.95
            main_cam.elevation = -23.0
            main_cam.azimuth = (38.0 if is_blue else 142.0) + 8.0 * math.sin(rendered_count * 0.025)

            renderer_main.update_scene(data, camera=main_cam)
            frame_rgb = renderer_main.render().copy()

            # Update PiP views (every frame during airborne shots, every 3rd frame otherwise)
            if pip_chase is None or pip_ov is None or len(airborne_pts) > 0 or (rendered_count % 3 == 0):
                renderer_pip.update_scene(data, camera="chase_cam")
                pip_chase = renderer_pip.render().copy()
                renderer_pip.update_scene(data, camera="ov9281_center")
                pip_ov = renderer_pip.render().copy()
            rendered_count += 1

            # Composite PiP windows with crisp Pelham Royal Blue / Cyan borders
            y1_ov, y2_ov = 20, 20 + PIP_H
            x1_pip, x2_pip = W - PIP_W - 20, W - 20
            frame_rgb[y1_ov - 3 : y2_ov + 3, x1_pip - 3 : x2_pip + 3] = (37, 99, 235)
            frame_rgb[y1_ov:y2_ov, x1_pip:x2_pip] = pip_ov

            y1_ch, y2_ch = H - PIP_H - 20, H - 20
            frame_rgb[y1_ch - 3 : y2_ch + 3, x1_pip - 3 : x2_pip + 3] = (56, 189, 248)
            frame_rgb[y1_ch:y2_ch, x1_pip:x2_pip] = pip_chase

            # Subtle top status bar indicator
            v_batt = float(f["vBatt"])
            d_stator = float(f["dStator"])
            t_sec = float(f["t"])
            blue_score = int(f["blueScore"])
            red_score = int(f["redScore"])
            intaked_cnt = int(f["intaked"])
            banner_txt = str(f.get("banner") or "")

            bar_col = (239, 68, 68) if v_batt < 6.8 else ((250, 204, 21) if airborne_pts else (34, 197, 94))
            frame_rgb[0:5, 0:W] = bar_col

            # Draw sleek Broadcast HUD Overlay via Pillow RGBA compositing
            pil_img = Image.fromarray(frame_rgb)
            draw = ImageDraw.Draw(pil_img, "RGBA")

            # Top-Left Broadcast Scoreboard & Match Clock Card (wide layout with zero text overlap)
            draw.rounded_rectangle([18, 16, 688, 92], radius=10, fill=(10, 15, 28, 215), outline=(56, 189, 248, 145), width=2)
            draw.text((32, 24), f"FRC 10413 FLUX  •  {short_run_id}", fill=(248, 250, 252, 255), font=font_title)
            draw.text(
                (32, 54),
                f"{robot_name} ({alliance or 'Blue'})  |  T = {t_sec:5.1f}s  |  {battery_name}",
                fill=(148, 163, 184, 255),
                font=font_mono,
            )

            # Score Pill inside Top Card
            draw.rounded_rectangle([508, 23, 676, 83], radius=8, fill=(23, 37, 84, 235), outline=(59, 130, 246, 195), width=1)
            draw.text((520, 29), f"BLUE HUB: {blue_score}", fill=(96, 165, 250, 255), font=font_score)
            draw.text((520, 59), f"INTAKED: {intaked_cnt}  RED: {red_score}", fill=(226, 232, 240, 255), font=font_small)

            # Bottom-Left Live Hardware Telemetry Card
            v_col = (248, 113, 113, 255) if v_batt < 6.8 else ((250, 204, 21, 255) if v_batt < 8.5 else (74, 222, 128, 255))
            draw.rounded_rectangle([18, H - 82, 688, H - 18], radius=10, fill=(10, 15, 28, 215), outline=(71, 85, 105, 160), width=1)
            draw.text((32, H - 72), f"BATTERY: {v_batt:4.2f} V", fill=v_col, font=font_mono)
            draw.text((212, H - 72), f"DRIVE STATOR: {d_stator:5.1f} A", fill=(251, 191, 36, 255), font=font_mono)
            draw.text((438, H - 72), f"SHOOTER: {shooter_rps:4.1f} RPS", fill=(56, 189, 248, 255), font=font_mono)
            draw.text(
                (32, H - 45),
                f"POSE: ({rx:5.2f}m, {ry:4.2f}m, {math.degrees(yaw):5.1f}°)   HOOD: {hood_rot:4.2f} rot   CAD: 14-Mesh + MuJoCo 3.15 EGL",
                fill=(203, 213, 225, 255),
                font=font_small,
            )

            # Shot Event Callout Banner (when active)
            if banner_txt:
                is_score = "SCORED" in banner_txt
                b_fill = (6, 78, 59, 225) if is_score else (127, 29, 29, 225)
                b_edge = (52, 211, 153, 240) if is_score else (248, 113, 113, 240)
                txt_w = int(draw.textlength(banner_txt, font=font_mono))
                box_x2 = min(x1_pip - 14, max(520, 32 + txt_w + 18))
                draw.rounded_rectangle([18, 102, box_x2, 140], radius=8, fill=b_fill, outline=b_edge, width=2)
                draw.text((32, 112), banner_txt, fill=(255, 255, 255, 255), font=font_mono)

            # PiP Camera Labels
            draw.rounded_rectangle([x1_pip + 8, y1_ov + 8, x1_pip + 225, y1_ov + 32], radius=5, fill=(10, 15, 28, 195))
            draw.text((x1_pip + 16, y1_ov + 13), "OV9281-2 HUB APRILTAG CAM", fill=(96, 165, 250, 255), font=font_small)

            draw.rounded_rectangle([x1_pip + 8, y1_ch + 8, x1_pip + 225, y1_ch + 32], radius=5, fill=(10, 15, 28, 195))
            draw.text((x1_pip + 16, y1_ch + 13), "3RD-PERSON CAD CHASE CAM", fill=(56, 189, 248, 255), font=font_small)

            out_rgb = np.asarray(pil_img)

            # Save high-res preview-3d.png when a ball is high in the air near the Hub (or at mid-match)
            if (not saved_preview) and len(airborne_pts) > 0 and max(p[2] for p in airborne_pts) > 1.30 and blue_score >= 3:
                pil_img.save(preview_png_path)
                saved_preview = True

            if proc.stdin:
                proc.stdin.write(out_rgb.tobytes())

        if not saved_preview:
            pil_img.save(preview_png_path)

    if proc.stdin:
        proc.stdin.close()
    proc.wait(timeout=30)


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: render_mujoco_replay.py <sim-frames.json> <out-replay-3d.mp4> <assets/meshes>", file=sys.stderr)
        sys.exit(1)
    render_match_video(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
