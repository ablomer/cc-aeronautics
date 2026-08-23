-- Ship-specific configuration.
--
-- This is the single source of truth for which physical peripheral (by its
-- ComputerCraft string ID) backs each logical role, and for geometry that
-- depends on how this hull is built. Other files should reference
-- PERIPHERALS.<SYSTEM>.<role> and SHIP.<SYSTEM>.<key> instead of hardcoding
-- those values, so that re-wiring or re-measuring the ship only requires
-- editing this file.
--
-- Each peripheral role is documented with a comment giving its type and a
-- short description, immediately above the string ID.

-- Hull / sensor geometry and ship-measured limits (not peripheral IDs).
SHIP = {
    LNAV = {
        -- Target velocity (m/s) at throttle lever 15. Lever 0 is stop.
        -- Tune after a flight if this is short of the hull's cruise.
        maxSpeed = 2.0,

        -- Propeller RSC cap. Create clamps setTargetSpeed to [-256, 256];
        -- this is the software ceiling the mixer and velocity loop share.
        maxRpm = 256,

        -- RPM differential at full steering lock (steer = ±1). At maxRpm
        -- the props fully counter-rotate for the fastest pivot, using the
        -- whole -256..256 range. The mixer gives this first claim on the
        -- budget and surge takes what is left, so turn authority does not
        -- fall off with throttle. Lower this to soften the heading loop:
        -- it scales RPM-per-degree without touching HeadingHold's gains.
        maxSteerDiff = 128,

        -- RPM differential while holding heading (wheel centered). The
        -- full maxSteerDiff range is for commanded turns; using it to
        -- chase a few degrees of error just weaves. 48 RPM is enough
        -- to trim a biased hull at hover without spinning the props past ~50.
        maxHoldDiff = 48,

        -- RPM differential for a persistent hold error (beyond HOLD_FAR).
        -- Cruise veer is stronger than hover; this is the extra trim
        -- budget so a 10° droop can still close. Near-target taper
        -- still keys off maxHoldDiff, so hover weave stays gone.
        maxHoldFarDiff = 120,

        -- Positive RSC RPM is backward on this hull; invert so +command is forward.
        invertLeft = true,
        invertRight = true,

        -- Flip if the ship spins continuously instead of settling, or
        -- the wheel yaws the hull the wrong way. Positive steer should
        -- yaw right.
        invertSteer = true,

        -- Wheel angles within this many degrees of center read as 0.
        -- Applied before normalizing to [-1, 1].
        steeringDeadzone = 1.0,

        -- Deg/s of heading-setpoint advance at full wheel lock.
        -- Tune down if full lock routinely trips maxHeadingLead.
        maxTurnRate = 20.0,

        -- Power-curve exponent on |normalized wheel|. 1 is linear;
        -- >1 gives finer control near center.
        turnRateExponent = 1.7,

        -- Max heading error (deg) the wheel-rate command is allowed
        -- to grow. Always permits movement that reduces the error.
        maxHeadingLead = 25.0,
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
    -- LNAV: horizontal navigation (forward velocity + steering)
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
        -- Detent 0 is stop. Driven to 0 via setSignal when nav arrives.
        throttleLever = "throttle_lever_9",

        -- type: velocity_sensor
        -- Reports current ship velocity, used by VelocityHold and shown on the display.
        velocitySensor = "velocity_sensor_4",

        -- type: steering_wheel
        -- Turn-rate command for heading hold when NAV is not steering.
        steeringWheel = "steering_wheel_5",

        -- type: navigation_table
        -- Heading + compass bearing/distance; drives HeadingHold in NAV mode.
        navigationTable = "navigation_table_2",
    },

    -- ------------------------
    -- VNAV: vertical navigation (altitude hold + landing)
    -- ------------------------
    VNAV = {
        -- type: throttle_lever
        -- Lever 1-15 maps linearly onto the altitude range; detent 0 is land
        -- (fixed sink, then optical flare).
        burnerLever = "throttle_lever_10",

        -- type: altitude_sensor
        -- Reports current height and vertical speed; height feeds the altitude
        -- outer loop, vertical speed is tracked by VerticalSpeedHold.
        altitudeSensor = "altitude_sensor_3",

        -- type: hot_air_burner (list)
        -- Heat sources; BurnerBank fans the commanded amount out to every burner in this list.
        burners = {
            "hot_air_burner_3",
        },

        -- type: analog_transmission
        -- Drives all vertical propellers together (single shared transmission).
        -- Leftover +up boost when VerticalSpeedHold is short of desiredVS.
        verticalPropellerTransmission = "analog_transmission_12",

        -- type: optical_sensor (list)
        -- Downward sensors; worst-case (closest hasHit) AGL drives cruise
        -- terrain climb and the landing flare. Never wired to an actuator.
        opticalSensors = {
            "optical_sensor_1",
        },
    },

    -- ------------------------
    -- ATT: attitude (gimbal pitch hold via the horizontal stabilizer)
    -- ------------------------
    ATT = {
        -- type: gimbal_sensor
        -- Reports body-frame pitch/roll (getAngles) and rates (getAngularRates).
        -- Pitch is xAngle, rotation about body-X; 0 = level.
        gimbalSensor = "gimbal_sensor_1",

        -- type: Create rotational speed controller
        -- Drives the stabilizer mechanical bearing. setTargetSpeed is integer RPM.
        stabilizerSpeedController = "Create_RotationSpeedController_3",

        -- type: Create mechanical bearing
        -- Reports the current stabilizer angle in degrees (positive = up).
        stabilizerBearing = "Create_MechanicalBearing_1",
    },

    -- ------------------------
    -- AUDIO: noteblock cues for mode changes and alerts
    -- ------------------------
    AUDIO = {
        -- type: speaker (list)
        -- Every speaker plays the same playNote cues for LNAV/VNAV
        -- transitions, nav acquire/lost, terrain warnings, and faults.
        speakers = {
            "speaker_1",
        },
    },

    -- ------------------------
    -- DEBUG: peripherals only exercised by debug.lua's standalone menu
    -- ------------------------
    DEBUG = {
        -- type: laser_pointer
        -- Introspected live via debug.lua's 'Laser sensor' menu entry.
        laserSensor = "laser_pointer_2",

        -- type: optical_sensor
        -- Same physical peripheral as VNAV.opticalSensors[1]; introspected via debug.lua's 'Optical sensor' menu entry.
        opticalSensor = "optical_sensor_1",

        -- type: gimbal_sensor
        -- Same physical peripheral as ATT.gimbalSensor; introspected via debug.lua's 'Gimbal sensor' menu entry.
        gimbalSensor = "gimbal_sensor_1",

        -- type: navigation_table
        -- Same physical peripheral as LNAV.navigationTable; introspected via debug.lua's 'Navigation table' menu entry.
        navigationTable = "navigation_table_2",
    },
}
