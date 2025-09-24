import config
import servo_map as servo
import time
import utilities as util

def pick_target(cal_az, cal_el, last_servo_az, last_servo_el):
    """Return (tgt_phys_az, tgt_phys_el, used_flip) after computing A/B options & costs."""
    # --- Build the two candidate poses in WORLD/PHYS/SE RVO space ---
    # A: normal

    if not config.ALLOW_BACKSIDE_FLIP:
        A_phys_az, A_phys_el = util.world_to_physical(cal_az, cal_el)
        return A_phys_az, A_phys_el, False

    A_world_az, A_world_el = cal_az, cal_el

    # B: backside (180° az shift; keep or mirror EL per your setting)
    B_world_az = util.wrap360(cal_az + 180.0)
    if config.FLIP_STYLE == "mirror_el":
        B_world_el = max(config.EL_PHYS_MIN, min(config.EL_PHYS_MAX, 180.0 - cal_el))
    else:
        B_world_el = cal_el

    # Small per-flip nudges (applied only if we actually flip later)
    def _apply_flip_corr(az, el):
        return util.wrap360(az + config.FLIP_AZ_CORR_DEG), max(config.EL_PHYS_MIN, min(config.EL_PHYS_MAX, el + config.FLIP_EL_CORR_DEG))

    # Map to PHYSICAL then SERVO for cost calc
    A_phys_az, A_phys_el = util.world_to_physical(A_world_az, A_world_el)
    B_phys_az, B_phys_el = util.world_to_physical(B_world_az, B_world_el)
    A_saz, A_sel = servo.physical_to_servo_deg(A_phys_az, A_phys_el)
    B_saz, B_sel = servo.physical_to_servo_deg(B_phys_az, B_phys_el)

    # Costs in SERVO degrees (what your motors actually move)
    cost_A = config.AZ_WEIGHT * abs(A_saz - last_servo_az) + config.EL_WEIGHT * abs(A_sel - last_servo_el)
    cost_B = config.AZ_WEIGHT * abs(B_saz - last_servo_az) + config.EL_WEIGHT * abs(B_sel - last_servo_el)

    # Respect horizon guard if you use it
    if B_world_el < config.MIN_EL_FOR_FLIP:
        used_flip = False
        return A_phys_az, A_phys_el, used_flip

    # Don’t allow flips far from seam 
    only_near_edge = config.ONLY_FLIP_NEAR_EDGE

    # Decide flip using the patched chooser
    used_flip, _chosen_saz = choose_flipped_if_better(A_saz, B_saz, cost_A, cost_B, only_flip_near_edge=only_near_edge)

    if used_flip:
        # Apply small corrections only when committing to flip
        B_world_az_corr, B_world_el_corr = _apply_flip_corr(B_world_az, B_world_el)
        B_phys_az, B_phys_el = util.world_to_physical(B_world_az_corr, B_world_el_corr)
        return B_phys_az, B_phys_el, True
    else:
        return A_phys_az, A_phys_el, False


def choose_flipped_if_better(A_saz, B_saz, cost_A, cost_B, only_flip_near_edge=True):
    """
    Decide whether to flip sides. Returns (use_flip: bool, chosen_saz: float)

    A_saz: current-side azimuth solution (deg)
    B_saz: flipped-side azimuth solution (deg)
    cost_A, cost_B: your existing cost values for A and B
    only_flip_near_edge: keep your existing behavior of allowing flips
                         only near the 0°/180° seam when True
    """
    now = time.time()

    # persistent state stored on the function itself
    st = choose_flipped_if_better.__dict__
    last_flip_t = st.get("_last_flip_t", 0.0)
    last_flip_saz = st.get("_last_flip_saz", None)

    def wrap180(x):
        # wrap to [-180, 180)
        return ((x + 180.0) % 360.0) - 180.0

    def near_edge(saz: float) -> bool:
    # seam is at 0° and 180° in SERVO space
        return (saz <= config.EDGE) or (saz >= (180.0 - config.EDGE))

    # Gate flipping
    allow_flip = True
    if only_flip_near_edge:
        allow_flip = near_edge(A_saz) or near_edge(B_saz)

    # time-based cooldown
    if (now - last_flip_t) < config.MIN_FLIP_DWELL_S:
        allow_flip = False

    # movement-based cooldown (don't re-flip until we moved away from seam)
    if last_flip_saz is not None:
        if abs(B_saz - last_flip_saz) < config.MIN_AZ_DELTA_SINCE_FLIP_DEG:
            allow_flip = False

    # Stronger "stickiness": add hysteresis + extra margin to B
    thresholded_cost_B = cost_B + config.FLIP_HYSTERESIS_DEG + config.FLIP_EXTRA_MARGIN_DEG

    if allow_flip and (thresholded_cost_B < cost_A):
        # commit to flip and remember time/angle
        st["_last_flip_t"] = now
        st["_last_flip_saz"] = B_saz
        return True, B_saz
    else:
        return False, A_saz
