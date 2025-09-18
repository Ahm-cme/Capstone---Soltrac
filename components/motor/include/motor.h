#pragma once
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Motor driver (2x linear actuators via PWM + DIR)                     │
    │ - AZ (azimuth) actuator and EL (elevation) actuator                   │
    │ - We don't have position encoders → purely "time of motion" control.  │
    │ - We map degrees ↔ mm using a simple linear model:                    │
    │     angle_deg / max_angle_deg = stroke_mm_used / stroke_mm_total      │
    │   This is an approximation but good enough for coarse solar pointing. │
    └───────────────────────────────────────────────────────────────────────┘

    My mental model (future-me reminder):
    - Each actuator is just a DC motor driver with one DIR pin and one PWM pin.
    - I set DIR to choose extend vs retract, then drive PWM at a fixed duty.
    - Since we lack feedback, we compute how long to run based on speed (mm/s).
    - We always add a small time buffer so we don't under-shoot (see move_time_ms).

    Hardware expectations:
    - MD20A drivers (or similar): 12V linear actuators with PWM speed control.
    - LEDC (low speed) @ 5 kHz, 13-bit resolution. Duty 0..8191 (I use ~4096 ≈ 50%).
    - DIR pins go directly to the motor driver's DIR inputs (logic level).
    - AZ and EL motors may have opposite polarity; we handle "homing" direction
      in tracking (az_home_dir_level / el_home_dir_level).

    Actuator specs (project-specific):
    - 12V 200mm stroke, rated 11.938 mm/s @ 12V (varies with battery voltage).
    - Force: enough to move solar panels in moderate wind (exact force TBD).
    - Duty cycle: these aren't designed for continuous operation, but solar
      tracking moves are brief (few seconds) with long pauses (5-15 min).

    Coordinate system notes:
    - "Panel degrees" = our logical coordinate system where:
        - AZ: 0° = some reference direction (North after calibration)
        - EL: 0° = horizon, 90° = zenith
    - "Actuator mm" = mechanical stroke position:
        - 0 mm = fully retracted, stroke_mm = fully extended
    - The angle-to-mm mapping is LINEAR, which assumes the linkage geometry
      gives roughly constant degrees-per-mm. Real linkages are nonlinear,
      but this is close enough for solar tracking (±2-3° accuracy is fine).

    Open-loop error sources (and why they matter):
    - Battery voltage sag: actuator slows down → undershoot target.
    - Temperature: affects motor torque and internal friction.
    - Wind load: panels catch wind, adding resistance.
    - Mechanical wear: backlash in linkages accumulates over time.
    - Time quantization: our vTaskDelay() isn't perfectly precise.

    Mitigation strategy:
    - Daily homing to mechanical hard stops resets accumulated error.
    - Move time buffer (300 ms) errs toward overshoot rather than undershoot.
    - 10° tracking tolerance means small errors don't trigger unnecessary moves.

    Note:
    - Speed depends on battery voltage and load → angles will drift during the day.
      We mitigate drift by homing to hard stops at sunset.
    - If you flip motor wires, the logical "extend" vs "retract" flips. Use
      tracking's homing DIR levels instead of changing math here.
    - Future improvement: add current sensing to detect stalls or end-of-travel.
*/

typedef struct {
    // PWM + DIR GPIOs for each actuator
    int az_pwm_pin; int az_dir_pin;     // Azimuth actuator (base rotation)
    int el_pwm_pin; int el_dir_pin;     // Elevation actuator (panel tilt)

    // Mechanism geometry + kinematics
    double stroke_mm;       // Full stroke length of actuator (e.g., 200 mm)
                           // This should match your actuator's datasheet spec.

    double speed_mm_per_s;  // Nominal speed at 12V (datasheet: ~11.938 mm/s)
                           // In practice this varies ±20% with voltage and load.
                           // We use this for time-of-motion calculations.

    // Logical angle limits in "panel space"
    // These define the usable range of each axis in our coordinate system.
    double max_az_deg;      // e.g., 270° usable azimuth range (0° to max_az_deg)
                           // Don't exceed actuator stroke or hit mechanical limits.

    double max_el_deg;      // e.g., 85° maximum elevation (nearly vertical)
                           // Limited by actuator stroke and structural clearance.

    double min_el_deg;      // e.g., 10° minimum elevation (avoid ground/mounts)
                           // Set high enough to avoid hitting base or supports.

    // Future expansion slots (not implemented yet):
    // bool az_dir_invert;  // Flip DIR polarity if wiring is backwards
    // bool el_dir_invert;  // Flip DIR polarity if wiring is backwards
    // int current_sense_az_pin; // ADC pin to monitor actuator current
    // int current_sense_el_pin; // ADC pin to monitor actuator current
} motor_cfg_t;

/*
    Initialize the motor subsystem:
    - Configure LEDC timer and two PWM channels (AZ, EL).
    - Set DIR pins as GPIO outputs.
    - Log the configuration for debugging.

    LEDC configuration details:
    - Timer: LEDC_TIMER_0, 5 kHz, 13-bit resolution (0-8191 duty range)
    - Channels: LEDC_CHANNEL_0 (AZ), LEDC_CHANNEL_1 (EL)
    - Speed mode: LEDC_LOW_SPEED_MODE (more stable than high-speed mode)

    Returns:
    - ESP_OK on success
    - ESP error codes on failure (GPIO/LEDC config issues)
*/
esp_err_t motor_init(const motor_cfg_t *cfg);

/*
    Move azimuth axis from current_deg to target_deg (open-loop):

    Algorithm:
    1) Clamp target_deg to [0, max_az_deg] for safety.
    2) Convert both current and target angles to "mm along stroke" using linear map.
    3) Determine direction: if target_mm > current_mm, extend (DIR=1), else retract (DIR=0).
    4) Compute move time: distance_mm / speed_mm_per_s + 300ms safety buffer.
    5) Start PWM at 50% duty (4096/8191), wait computed time, stop PWM.

    Important notes:
    - This function BLOCKS the calling task for the entire move duration.
    - No position feedback; we trust the time calculation and hope for the best.
    - The 300ms buffer helps ensure we reach or slightly overshoot the target.
    - DIR polarity assumes your wiring: DIR=1→extend, DIR=0→retract.

    Parameters:
    - current_deg: where we think the axis is now (from tracking state)
    - target_deg: where we want it to go (computed from sun position)

    Side effects:
    - Drives AZ actuator for several seconds
    - Logs move details (helpful for debugging tracking accuracy)
    - Updates internal PWM state
*/
void motor_move_az(double current_deg, double target_deg);

/*
    Move elevation axis from current_deg to target_deg (open-loop):

    Same algorithm as motor_move_az, but:
    - Clamps to [min_el_deg, max_el_deg] instead of [0, max_az_deg]
    - Uses EL PWM channel and EL DIR pin
    - Elevation is typically more constrained (avoid hitting ground/mounts)

    Elevation-specific considerations:
    - Gravity affects this axis differently than azimuth.
    - Wind load on tilted panels can be significant.
    - Mechanical limits are often tighter (don't hit the ground).

    Parameters:
    - current_deg: current elevation angle (horizon = 0°, zenith = 90°)
    - target_deg: desired elevation angle
*/
void motor_move_el(double current_deg, double target_deg);

/*
    Park both axes to a safe overnight position:
    - Convenience wrapper that moves AZ then EL sequentially.
    - Typically called before deep sleep or in high wind conditions.
    - Uses the same open-loop motion as individual axis moves.

    Common park positions:
    - Face East (AZ ≈ 90°) at moderate elevation (EL ≈ 45°) for morning startup.
    - Face down (EL ≈ 10°) in high wind to reduce load.
    - Face away from prevailing weather direction.

    Parameters:
    - park_az_deg, park_el_deg: desired park position in panel degrees
    - cur_az, cur_el: current position estimate (for time calculation)

    Duration: sum of both individual moves, so potentially 10-30 seconds total.
*/
void motor_park(double park_az_deg, double park_el_deg, double cur_az, double cur_el);

/*
    Emergency stop: immediately set both PWM channels to 0% duty.
    - DIR levels are unchanged (still show last commanded direction).
    - Useful for fault conditions, overvoltage protection, etc.
    - Actuators will coast to a stop (no dynamic braking).

    Use cases:
    - User button abort sequence
    - Battery voltage too low/high
    - System fault or watchdog timeout
    - Manual override during testing
*/
void motor_stop_all(void);

/*
    Low-level timed run utilities (used by "homing to mechanical stops"):

    These bypass the angle-to-mm conversion and just run raw PWM + DIR
    for a specified duration. Used by the tracking system to drive actuators
    to their mechanical limits for daily position reset.

    Parameters:
    - dir_level: raw logic level written to DIR pin (0 or 1)
    - ms: duration in milliseconds to run at full PWM

    Behavior:
    1) Set DIR pin to dir_level immediately
    2) Start PWM at 4096/8191 (≈50% duty)
    3) Wait exactly 'ms' milliseconds
    4) Stop PWM (actuator coasts to a stop)

    Safety considerations:
    - These functions can drive actuators past their normal angle limits.
    - Only use for homing against known mechanical hard stops.
    - Ensure 'ms' is long enough to guarantee hitting the stop.
    - Actuators may draw high current when stalled against stops.

    Homing strategy:
    - Drive each axis toward its mechanical limit for longer than needed.
    - The actuator will stall when it hits the hard stop.
    - After homing, we assign a known angle to that mechanical position.
    - This eliminates accumulated open-loop error each day.
*/
void motor_run_az_ms(int dir_level, uint32_t ms);
void motor_run_el_ms(int dir_level, uint32_t ms);