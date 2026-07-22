"""Nominal (no-noise) margin search for Following scenarios: sweep starting
gaps behind a steady lead and report each fate, locating the crash/safe
boundary around which the Following scenario states will be placed."""
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from system.vehicle_plant import VehiclePlant
from system.controller import AEBSController
from system.perception import PerceptionSystem
from system.environment import TrafficEnvironment

DT = 0.1
LEAD_V = 10.0
STEPS = 600

print(f"{'ego_v':>6} {'gap0':>6} {'fate':>8} {'min_gap':>8} {'t_end':>6}")
for ego_v in (12.0, 14.0, 16.0):
    for gap0 in [round(4 + 0.5 * i, 1) for i in range(45)]:   # 4.0 .. 26.0 m
        env = TrafficEnvironment(dt=DT)
        perception = PerceptionSystem(false_negative_rate=0.0)
        controller = AEBSController(mode='industry', lead_behavior='steady')
        plant = VehiclePlant(dt=DT, coordinate_system='world_frame')
        env.configure(scenario_type='steady', initial_gap=gap0,
                      initial_ego_v=ego_v, initial_lead_v=LEAD_V)
        plant.actual_velocity = ego_v
        min_gap, fate, t_end = gap0, 'settle', STEPS * DT
        for step in range(STEPS):
            ego_disp = plant.actual_velocity * DT
            gt = env.update_physics(ego_disp, plant.actual_velocity)
            seen, og, ov = perception.read_sensors(gt)
            acc, _ = controller.get_action(seen, og, ov)
            plant.step(acc)
            min_gap = min(min_gap, gt['gap'])
            if gt['gap'] <= 0:
                fate, t_end = 'CRASH', step * DT
                break
        print(f"{ego_v:>6.1f} {gap0:>6.1f} {fate:>8} {min_gap:>8.2f} {t_end:>6.1f}", flush=True)
