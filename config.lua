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

        -- Positive RSC RPM is backward on this hull; invert so +command is forward.
        invertLeft = true,
        invertRight = true,

        -- Flip if nav locks at ~180° (inverted heading loop) or the wheel
        -- yaws the hull the wrong way. Positive steer should yaw right.
        invertSteer = false,

        -- Wheel angles within this many degrees of center read as 0.
        steeringDeadzone = 1.0,
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
        rightPropellerSpeedController = "Create_RotationSpeedController_2",

        -- type: Create rotational speed controller
        -- Drives the left propeller. setTargetSpeed is integer RPM in [-256, 256].
        leftPropellerSpeedController = "Create_RotationSpeedController_1",

        -- type: throttle_lever
        -- Velocity setpoint: position 0-15 maps onto 0 .. SHIP.LNAV.maxSpeed.
        -- Detent 0 is stop. Driven to 0 via setSignal when nav arrives.
        throttleLever = "throttle_lever_7",

        -- type: velocity_sensor
        -- Reports current ship velocity, used by VelocityHold and shown on the display.
        velocitySensor = "velocity_sensor_3",

        -- type: steering_wheel
        -- Manual steering input, read whenever nav (bearing hold) steering is not engaged.
        steeringWheel = "steering_wheel_3",

        -- type: navigation_table
        -- Provides nav target bearing/heading/distance; drives BearingHold autopilot steering.
        navigationTable = "navigation_table_1",
    },

    -- ------------------------
    -- VNAV: vertical navigation (altitude hold + landing)
    -- ------------------------
    VNAV = {
        -- type: throttle_lever
        -- Lever 1-15 maps linearly onto the altitude range; detent 0 is land
        -- (fixed sink, then optical flare).
        burnerLever = "throttle_lever_8",

        -- type: altitude_sensor
        -- Reports current height and vertical speed; height feeds the altitude
        -- outer loop, vertical speed is tracked by VerticalSpeedHold.
        altitudeSensor = "altitude_sensor_2",

        -- type: hot_air_burner (list)
        -- Heat sources; BurnerBank fans the commanded amount out to every burner in this list.
        burners = {
            "hot_air_burner_2",
        },

        -- type: analog_transmission
        -- Drives all vertical propellers together (single shared transmission).
        -- Leftover +up boost when VerticalSpeedHold is short of desiredVS.
        verticalPropellerTransmission = "analog_transmission_11",

        -- type: optical_sensor (list)
        -- Downward sensors; worst-case (closest hasHit) AGL drives cruise
        -- terrain climb and the landing flare. Never wired to an actuator.
        opticalSensors = {
            "optical_sensor_0",
        },
    },

    -- ------------------------
    -- ATT: attitude (gimbal pitch hold via the horizontal stabilizer)
    -- ------------------------
    ATT = {
        -- type: gimbal_sensor
        -- Reports body-frame pitch/roll (getAngles) and rates (getAngularRates).
        -- Pitch is xAngle, rotation about body-X; 0 = level.
        gimbalSensor = "gimbal_sensor_0",

        -- type: Create rotational speed controller
        -- Drives the stabilizer mechanical bearing. setTargetSpeed is integer RPM.
        stabilizerSpeedController = "Create_RotationSpeedController_0",

        -- type: Create mechanical bearing
        -- Reports the current stabilizer angle in degrees (positive = up).
        stabilizerBearing = "Create_MechanicalBearing_0",
    },

    -- ------------------------
    -- AUDIO: noteblock cues for mode changes and alerts
    -- ------------------------
    AUDIO = {
        -- type: speaker (list)
        -- Every speaker plays the same playNote cues for LNAV/VNAV
        -- transitions, nav acquire/lost, terrain warnings, and faults.
        speakers = {
            "speaker_0",
        },
    },

    -- ------------------------
    -- DEBUG: peripherals only exercised by debug.lua's standalone menu
    -- ------------------------
    DEBUG = {
        -- type: laser_pointer
        -- Introspected live via debug.lua's 'Laser sensor' menu entry.
        laserSensor = "laser_pointer_1",

        -- type: optical_sensor
        -- Same physical peripheral as VNAV.opticalSensors[1]; introspected via debug.lua's 'Optical sensor' menu entry.
        opticalSensor = "optical_sensor_0",

        -- type: gimbal_sensor
        -- Same physical peripheral as ATT.gimbalSensor; introspected via debug.lua's 'Gimbal sensor' menu entry.
        gimbalSensor = "gimbal_sensor_0",
    },
}
