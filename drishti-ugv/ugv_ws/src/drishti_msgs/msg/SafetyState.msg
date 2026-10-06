# Safety supervisor decision for one tick, published on /safety/state.
#
# SPEC.md section 9. The supervisor is the only publisher on /cmd_vel and it
# never increases a commanded velocity. This message is the audit record of
# why it did what it did, and is logged with every run.

std_msgs/Header header

# ---- action taken -------------------------------------------------------
uint8 ACTION_PASS = 0            # Nav2 command forwarded unchanged
uint8 ACTION_SLOW = 1            # forwarded, clamped to v_slow
uint8 ACTION_STOP = 2            # zero velocity published

uint8 action

# ---- why ----------------------------------------------------------------
# These values match drishti_safety::Reason one for one; they follow the
# SPEC.md section 9.1 *listing* order.
#
# They are NOT the evaluation order. The supervisor checks every STOP condition
# before the SLOW condition, so LOW_CONFIDENCE (5) is evaluated after
# PATH_INVALID (6) and COMMAND_INVALID (7). See the comment on
# SupervisorCore::evaluate for why, and do not infer ordering from these
# numbers.
uint8 REASON_NONE = 0
uint8 REASON_LOCALIZATION_LOST = 1
uint8 REASON_DEPTH_STALE = 2
uint8 REASON_CAMERA_STALE = 3
uint8 REASON_OBSTACLE_EMERGENCY = 4
uint8 REASON_LOW_CONFIDENCE = 5
uint8 REASON_PATH_INVALID = 6
uint8 REASON_COMMAND_INVALID = 7
# Frames keep arriving with fresh timestamps but the content has stopped
# changing. Distinct from CAMERA_STALE, which is silence: a frozen camera
# looks perfectly alive, which is what makes it dangerous (D18).
uint8 REASON_CAMERA_FROZEN = 8

uint8 reason
string detail                    # human-readable, for logs and the operator view

# ---- what was published on /cmd_vel ------------------------------------
bool stop                        # mirrors /safety/stop this tick
float32 v_limit                  # m/s ceiling applied to the outgoing command
float32 commanded_linear_x       # m/s actually published
float32 commanded_angular_z      # rad/s actually published

# ---- evidence behind the decision --------------------------------------
# Recorded even when action is ACTION_PASS, so a run can be replayed and the
# decision boundary inspected without re-running perception.
float32 rgb_age                  # s since last RGB frame
float32 rgb_static_for           # s the RGB content has been unchanged
float32 depth_age                # s since last depth frame
float32 pose_covariance_max      # largest diagonal term considered
float32 nearest_obstacle         # m; NaN when unknown
float32 perception_confidence    # 0..1
