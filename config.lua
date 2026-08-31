-- Ship-specific configuration.
--
-- This is the single source of truth for peripherals that share a type
-- with another role (so they must be named by ComputerCraft string ID)
-- and for geometry that depends on how this hull is built. Other files
-- should reference PERIPHERALS.<SYSTEM>.<role> and SHIP.<SYSTEM>.<key>
-- instead of hardcoding those values, so that re-wiring or re-measuring
-- the ship only requires editing this file.
--
-- Unique-type peripherals are discovered at startup with
-- peripheral.find and are not listed here:
--   exactly one: steering_wheel, navigation_table,
--                velocity_sensor, altitude_sensor, gimbal_sensor
--   one or more: hot_air_burner, optical_sensor
--   zero or more: speaker (DFPWM callouts from audio/; silent if none)
--
-- Each remaining peripheral role is documented with a comment giving
-- its type and a short description, immediately above the string ID.

-- Hull / sensor geometry and ship-measured limits (not peripheral IDs).
SHIP = {
    LNAV = {
        -- Target velocity (m/s) at throttle lever 15. Lever 0 is stop.
        -- Tune after a flight if this is short of the hull's cruise.
        maxSpeed = 6.5,

        -- Common-mode RPM at a speed request of 1.0 (straight flight).
        -- Match maxRpm so a full lever can use the whole engine. Turning
        -- takes RPM from this pool first; the speed loop's target and
        -- ceiling scale with whatever is left (see mixSpeedHeadroom).
        forwardRpm = 72,

        -- Differential RPM at a steering request of ±1.0. Match maxRpm
        -- so a full turn can use the whole engine (one side +max, the
        -- other -max when speed has been fully shed). Linear: a 0.5
        -- steer request leaves half the RPM for speed. This is yaw
        -- torque, so it also sets how hard the damper can brake a
        -- residual spin. Lower if small corrections feel like a pivot.
        turnRpm = 72,

        -- Actuator ceiling sent to each propeller RSC. Create clamps
        -- setTargetSpeed to [-256, 256]; this is the software cap the
        -- mixer and Propeller objects share. Measure nothing — it is
        -- the hardware limit unless a gearbox needs a lower software cap.
        maxRpm = 72,

        -- Relative bearing (deg) ignored as noise. Measure wheel slop
        -- at rest; keep this just above the idle wobble.
        steerDeadband = 2.0,

        -- Relative bearing (deg) that commands full turning authority.
        -- Smaller = snappier. Wheel only: deadband .. full angle -> 0 .. 1.
        -- NAV compass tracking uses navBearingGain / navMaxYawRate instead.
        steerFullAngle = 180.0,

        -- Yaw-rate damper. Simulated hulls keep spinning after the
        -- wheel recenters because differential thrust going to 0 does
        -- not cancel leftover angular velocity. This term brakes that
        -- residual. Units: normalized steer per deg/s of yaw-rate error.
        --
        -- Wheel still couples braking and sustained rate: full deflection
        -- settles at about 1/gain deg/s. NAV does not — it tracks a rate
        -- command capped at navMaxYawRate, so raising this gain still
        -- stops harder without slowing compass tracking at cruise.
        -- At 0.10: full braking above ~10 deg/s, full wheel holds a
        -- ~10 deg/s turn (360 in ~34 s). Lower for quicker wheel turns
        -- that coast longer; raise if the hull still drifts after
        -- centering, but back off if it hunts around straight.
        yawDampGain = 0.10,

        -- NAV compass: yaw-rate (deg/s) per degree of bearing error
        -- outside steerDeadband. At 0.25 a 40° offset wants 10 deg/s,
        -- matching today's full-wheel rate. Raise if a distant compass
        -- (almost a heading bug) still feels lazy; the close-in needle
        -- is handled by navLosGain.
        navBearingGain = 0.25,

        -- NAV compass: extra yaw-rate matching the needle's geometric
        -- swing, (v * sin(bearing) / range) in deg/s. 1.0 keeps the
        -- nose on a moving compass; raise toward 1.5 if cruise still
        -- lags a nearby lodestone. 0 disables lead and leaves only P.
        navLosGain = 1.0,

        -- Floor on nav-table range (m) so the LOS term cannot explode
        -- over the target. Tune near the tightest turn this hull makes.
        navMinRange = 12.0,

        -- Pull the speed lever to detent 0 when NAV ground range is at
        -- or inside this (metres). Height is not counted. Direct-to
        -- only; a holding pattern does not arrive. Stays parked while
        -- a live target is that close; clear the compass to leave.
        -- Wheel-only has no destination and is not affected.
        navArriveRange = 50.0,

        -- true = clockwise holding pattern (target on the right, +90°).
        -- false = left-hand orbit. Radius is not commanded: it follows
        -- speed (faster = larger circle). Click PTN to orbit; compass
        -- insert always returns to direct-to.
        navHoldClockwise = true,

        -- NAV yaw-rate ceiling (deg/s) at SHIP.LNAV.maxSpeed. Lerps
        -- from 1/yawDampGain at a stop up to this at cruise, so slow
        -- flight keeps today's wheel-like cap and high speed can
        -- match a swinging compass. 28 deg/s at 6.5 m/s is a ~13 m
        -- radius; raise if cruise still flies past the needle.
        navMaxYawRate = 28.0,

        -- Deg/s treated as already stopped. Keep just above gimbal noise.
        -- This is the residual creep the damper will not chase out, so
        -- it is a floor on how precisely a heading can be held: at 0.5
        -- the hull can still drift half a degree per second. Raise it
        -- if the props chatter around straight.
        yawDampDeadband = 0.5,

        -- Positive RSC RPM is backward on this hull; invert so +command is forward.
        invertLeft = true,
        invertRight = true,

        -- Compass heading of the nav table's 0° mark / block arrow
        -- (0 = north, 90 = east, 180 = south, 270 = west). Added to
        -- getHeading() after the south→north conversion and to
        -- getBearing(), then wrapped. This table's 0 points west;
        -- 0 if the arrow already faces north with the hull.
        navTableYaw = 90,

        -- Flip if a positive bearing (target / wheel to the right)
        -- yaws the hull left. Independent of invertLeft/Right.
        invertSteer = true,

        -- Sign of gimbal wy against a right turn. Independent of
        -- invertSteer: this is sensor polarity, not propeller wiring,
        -- so setting it by feel while also flipping invertSteer will
        -- chase its own tail. Test at throttle 0: start a turn, center
        -- the wheel, and watch the display. YAW and STR should end up
        -- opposite in sign as the hull brakes. Same sign (the turn
        -- accelerates into the clamp) means flip this.
        invertYawDamp = false,
    },
    VNAV = {
        -- Optical AGL (metres) when the hull is sitting on the ground.
        -- Flare keeps a residual sink through this height, then latches
        -- landed and cuts heat. Measure at rest, not in the hover.
        touchdownAgl = 2.0,

        -- Gravitational force on the hull (pN). F_down in the heated-volume
        -- formula: V = F_down * e^((H - 63) / 250) / 15.63. Read from the
        -- ship's physics overlay. Too high settles above the lever target;
        -- too low settles below it.
        downwardForce = 23688.50,

        -- Lowest cruise Y the burner lever commands (notch 1). Notch 15 is
        -- the max Y the balloon can hold given capacity and downwardForce.
        -- Notch 0 is land. Tune to the lowest altitude this hull should hover.
        minAltitude = 92,

        -- Heat-command slew (m³/s) at minAltitude. Notch-1..15 interpolates
        -- this .. maxVolumeRate against current Y between minAltitude and
        -- the balloon's max Y. Faster at high altitude, slower down low.
        minVolumeRate = 5.0,

        -- Heat-command slew (m³/s) at the balloon's max Y (lever 15).
        maxVolumeRate = 30.0,

        -- Heat-command slew (m³/s) used when dumping heat near the
        -- ground: any optical hasHit() and the command is decreasing.
        -- Takeoff (heat increasing) keeps the cruise lerp even with a hit.
        landingVolumeRate = 1.0,

        -- Lowest total heated volume (m³) the burners may command. Held
        -- on the ground after landing so the envelope stays inflated
        -- enough to keep the hull upright. Tune just below lift-off.
        minHeatedVolume = 1250.0,

        -- Actuator ceiling sent to the vertical propeller RSC. Create
        -- clamps setTargetSpeed to [-256, 256]; leftover climb boost
        -- saturates at this cap. Lower it if a gearbox needs a software
        -- limit; raise toward 256 if the boost is weak.
        maxRpm = 256,

        -- Flip if positive RPM pushes the hull down.
        invert = false,
    },
    ATT = {
        -- Stabilizer travel clamp (degrees). Positive is an up angle.
        minAngle = -45,
        maxAngle = 45,

        -- Speed cap well under the RSC's 256 RPM limit. 1 RPM is already
        -- 6 deg/s at the bearing before any gearing.
        maxRpm = 8,

        -- Degrees per second of bearing travel per 1 RPM. 360/60 = 6 at
        -- 1:1; multiply by the gear ratio if the RSC is not direct-drive.
        -- Calibrate from the debug script's measured deg/s.
        degPerSecPerRpm = 6.0,

        -- Flip if positive RPM decreases getAngle().
        invertServo = false,

        -- Flip if a positive stabilizer angle pitches the hull the wrong way.
        invertPitch = false,
    },
    AUDIO = {
        -- speaker.playAudio volume, 0.0-3.0. 3.0 is the peripheral max.
        volume = 1.0,
    },
}

PERIPHERALS = {
    -- ------------------------
    -- LNAV: speed hold + heading (shared left/right props)
    -- ------------------------
    LNAV = {
        -- type: Create rotational speed controller
        -- Drives the right propeller. setTargetSpeed is integer RPM in [-256, 256].
        rightPropellerSpeedController = "Create_RotationSpeedController_0",

        -- type: Create rotational speed controller
        -- Drives the left propeller. setTargetSpeed is integer RPM in [-256, 256].
        leftPropellerSpeedController = "Create_RotationSpeedController_1",

        -- type: throttle_lever
        -- Velocity setpoint: position 0-15 maps onto 0 .. SHIP.LNAV.maxSpeed.
        -- Detent 0 is stop.
        throttleLever = "throttle_lever_2",
    },

    -- ------------------------
    -- VNAV: vertical navigation (altitude hold, landing, terrain)
    -- ------------------------
    VNAV = {
        -- type: throttle_lever
        -- Detent 0 is land (slew heat toward minimum). The cruise slew
        -- lerp still applies until an optical hasHit() while descending,
        -- which switches to landingVolumeRate. Takeoff is not limited.
        -- Positions 1-15 interpolate minAltitude .. balloon max altitude.
        burnerLever = "throttle_lever_1",

        -- type: Create rotational speed controller
        -- Drives all vertical propellers together (single shared RSC).
        -- Parked at 0: vertical speed is a consequence of heat-volume
        -- slew, not leftover VS-boost on this RSC.
        -- setTargetSpeed is integer RPM in [-256, 256].
        verticalPropellerSpeedController = "Create_RotationSpeedController_2",
    },

    -- ------------------------
    -- ATT: attitude (gimbal pitch hold via the horizontal stabilizer)
    -- The gimbal is required at startup (LNAV yaw damping uses it too).
    -- The stabilizer RSC and bearing are optional: if either is missing,
    -- pitch hold is not run and the display shows ATT [ NONE ].
    -- ------------------------
    ATT = {
        -- type: Create rotational speed controller
        -- Drives the stabilizer mechanical bearing. setTargetSpeed is integer RPM.
        stabilizerSpeedController = "Create_RotationSpeedController_3",

        -- type: Create mechanical bearing
        -- Reports the current stabilizer angle in degrees (positive = up).
        stabilizerBearing = "Create_MechanicalBearing_1",
    },

    -- ------------------------
    -- DEBUG: peripherals only exercised by debug.lua's standalone menu
    -- ------------------------
    DEBUG = {
        -- type: laser_pointer
        -- Introspected live via debug.lua's 'Laser sensor' menu entry.
        laserSensor = "laser_pointer_2",
    },
}
