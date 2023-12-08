from typing import Tuple

def ddr_ik(v_x, omega, L=0.5, R=0.1) -> Tuple[float, float]:
    """DDR inverse kinematics: calculate wheel rps from desired velocity."""
    return ((v_x - (L/2)*omega)/R, (v_x + (L/2)*omega)/R)

def calc_velocity(wl, wr, L=0.5, R=0.1) -> Tuple[float,float,float]:
    """DDR inverse kinematics: calculate robot velocity from wheel rps"""
    return (R*(wr+wl)/2.0, 0, (R/2)*(wr-wl)/L)

def calc_rpm(ticks: int, time_elapsed: float, ticks_per_rev: int = 360) -> float:
    return (ticks / ticks_per_rev)/(time_elapsed/60.0)

def calc_rps(ticks: int, time_elapsed: float, ticks_per_rev: int = 360) -> float:
    return calc_rpm(ticks=ticks, time_elapsed=time_elapsed, ticks_per_rev=ticks_per_rev)/60.0


