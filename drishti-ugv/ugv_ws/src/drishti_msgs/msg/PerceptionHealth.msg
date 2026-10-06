# Perception liveness and quality, published on /perception/health.
#
# SPEC.md sections 5.3 and 9.2. The supervisor consumes this to decide whether
# perception can be trusted at all; it does not look inside the model.

std_msgs/Header header

# Sensor timestamps as carried by the source messages -- never now() at
# publish time (SPEC.md section 3.2 rule 3).
builtin_interfaces/Time last_rgb_stamp
builtin_interfaces/Time last_depth_stamp

float32 rgb_age                  # s since last RGB frame
float32 depth_age                # s since last depth frame
bool rgb_ok                      # rgb_age <= t_camera_stale
bool depth_ok                    # depth_age <= t_depth_stale

# Seconds the RGB content has been unchanged. Liveness is not enough: a frozen
# camera republishes the same image with a fresh stamp, so rgb_age stays small
# while the view of the world is minutes old. The supervisor stops on this
# independently of rgb_age (D18/D19).
float32 rgb_static_for

float32 mean_confidence          # 0..1 across this frame's outputs
float32 latency_ms               # frame arrival -> output published
uint16 detection_count
