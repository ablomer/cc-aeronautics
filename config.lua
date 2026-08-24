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
--   exactly one: steering_wheel, navigation_table, gimbal_sensor,
--                velocity_sensor, altitude_sensor
--   one or more: hot_air_burner, optical_sensor, speaker
--
-- Each remaining peripheral role is documented with a comment giving
-- its type and a short description, immediately above the string ID.

-- Hull / sensor geometry and ship-measured limits (not peripheral IDs).
SHIP = {
    LNAV = {
        -- Target velocity (m/s) at throttle lever 15. Lever 0 is stop.
        -- Tune after a flight if this is short of the hull's cruise.
        maxSpeed = 2.0,

        -- Common-mode RPM at a speed request of 1.0. Fly straight at lever
        -- 15: if hold cannot reach maxSpeed, raise this. Leave headroom
        -- so forwardRpm + turnRpm stays at or under maxRpm; otherwise
        -- the mixer will shed forward thrust to keep full turning authority.
        -- 192 + 64 = 256 uses this hull's actuator ceiling with no CUT.
        forwardRpm = 192,

        -- Differential RPM at a steering request of ±1.0. One side gets
        -- +turnRpm and the other -turnRpm on a pivot (speed request 0).
        -- Raise if turns are sluggish; lower if the hull yaws too hard.
        turnRpm = 64,

        -- Actuator ceiling sent to each propeller RSC. Create clamps
        -- setTargetSpeed to [-256, 256]; this is the software cap the
        -- mixer and Propeller objects share. Measure nothing — it is
        -- the hardware limit unless a gearbox needs a lower software cap.
        maxRpm = 256,

        -- Relative bearing (deg) ignored as noise. Measure wheel slop
        -- at rest; keep this just above the idle wobble.
        steerDeadband = 2.0,

        -- Relative bearing (deg) that commands full turning authority.
        -- Smaller = snappier. Wheel and nav-table bearings share this
        -- linear map: deadband .. full angle -> 0 .. 1.
        steerFullAngle = 45.0,

        -- Positive RSC RPM is backward on this hull; invert so +command is forward.
        invertLeft = true,
        invertRight = true,

        -- Flip if a positive bearing (target / wheel to the right)
        -- yaws the hull left. Independent of invertLeft/Right.
        invertSteer = false,
    },
    VNAV = {
        -- Optical AGL (metres) when the hull is sitting on the ground.
        -- Flare keeps a residual sink through this height, then latches
        -- landed and cuts heat. Measure at rest, not in the hover.
        touchdownAgl = 2.0,
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
}

PERIPHERALS = {
    -- ------------------------
    -- LNAV: speed hold + heading (shared left/right props)
    -- ------------------------
    LNAV = {
        -- type: Create rotational speed controller
        -- Drives the right propeller. setTargetSpeed is integer RPM in [-256, 256].
        rightPropellerSpeedController = "Create_RotationSpeedController_5",

        -- type: Create rotational speed controller
        -- Drives the left propeller. setTargetSpeed is integer RPM in [-256, 256].
        leftPropellerSpeedController = "Create_RotationSpeedController_4",

        -- type: throttle_lever
        -- Velocity setpoint: position 0-15 maps onto 0 .. SHIP.LNAV.maxSpeed.
        -- Detent 0 is stop.
        throttleLever = "throttle_lever_11",
    },

    -- ------------------------
    -- VNAV: vertical navigation (altitude hold + landing)
    -- ------------------------
    VNAV = {
        -- type: throttle_lever
        -- Lever 1-15 maps linearly onto the altitude range; detent 0 is land
        -- (fixed sink, then optical flare).
        burnerLever = "throttle_lever_10",

        -- type: analog_transmission
        -- Drives all vertical propellers together (single shared transmission).
        -- Leftover +up boost when VerticalSpeedHold is short of desiredVS.
        verticalPropellerTransmission = "analog_transmission_12",
    },

    -- ------------------------
    -- ATT: attitude (gimbal pitch hold via the horizontal stabilizer)
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
