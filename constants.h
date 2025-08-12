// Wheel -> sprocket
constexpr float WHEEL_RADIUS = 0.025f; // sprocket[m]
constexpr float WHEEL_BASE = 0.1874f;  // [m] center-to-center
constexpr int TICKS_PER_REV = 462;     // reduction_ratio * ppr_num
constexpr float DIST_PER_TICKS = (2 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
constexpr int ODOM_PERIOD_MS = 100;     // TODO - Tune this.
constexpr float MAX_WHEEL_SPEED = 1.0f; // Max wheel linear speed in m/s
