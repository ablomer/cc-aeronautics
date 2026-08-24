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
--   zero or more: speaker (audio is silent if none are attached)
--
-- Each remaining peripheral role is documented with a comment giving
-- its type and a short description, immediately above the string ID.

-- Hull / sensor geometry and ship-measured limits (not peripheral IDs).
SHIP = {
    LNAV = {
        -- Target velocity (m/s) at throttle lever 15. Lever 0 is stop.
        -- Tune after a flight if this is short of the hull's cruise.
        maxSpeed = 5.0,

        -- Common-mode RPM at a speed request of 1.0. Fly straight at lever
        -- 15: if hold cannot reach maxSpeed, raise this. Leave headroom
        -- so forwardRpm + turnRpm stays at or under maxRpm; otherwise
        -- the mixer will shed forward thrust to keep full turning authority.
        -- 96 + 48 = 144 is over the 128 ceiling, so a full-deflection
        -- turn at cruise drops common-mode to 80 and the display shows
        -- CUT. Deliberate: braking authority matters more than the last
        -- 16 RPM of forward thrust while turning.
        forwardRpm = 96,

        -- Differential RPM at a steering request of ±1.0. One side gets
        -- +turnRpm and the other -turnRpm on a pivot (speed request 0).
        -- This is yaw torque, so it sets how fast the hull reaches a
        -- turn rate and how hard the damper can brake one, but not the
        -- sustained rate itself (that is 1/yawDampGain). Raise to cut
        -- overshoot when centering; lower if the hull snaps too hard.
        -- Above maxRpm - forwardRpm the mixer starts shedding forward
        -- thrust in hard turns and the display shows CUT.
        turnRpm = 48,

        -- Actuator ceiling sent to each propeller RSC. Create clamps
        -- setTargetSpeed to [-256, 256]; this is the software cap the
        -- mixer and Propeller objects share. Measure nothing — it is
        -- the hardware limit unless a gearbox needs a lower software cap.
        maxRpm = 144,

        -- Relative bearing (deg) ignored as noise. Measure wheel slop
        -- at rest; keep this just above the idle wobble.
        steerDeadband = 2.0,

        -- Relative bearing (deg) that commands full turning authority.
        -- Smaller = snappier. Wheel and nav-table bearings share this
        -- linear map: deadband .. full angle -> 0 .. 1.
        steerFullAngle = 45.0,

        -- Yaw-rate damper. Simulated hulls keep spinning after the
        -- wheel recenters because differential thrust going to 0 does
        -- not cancel leftover angular velocity. This term brakes that
        -- residual. Units: normalized steer per deg/s of yaw rate.
        --
        -- This gain does double duty, so read both effects before
        -- retuning:
        --   Braking. A residual rate reaches full counter-thrust
        --   (turnRpm) once it exceeds 1/gain deg/s. Below that the
        --   brake is proportional and weaker.
        --   Sustained turn rate. Holding full wheel settles where the
        --   damper cancels the request, at about 1/gain deg/s. Raising
        --   the gain to stop faster also makes turns slower, and
        --   nothing here can decouple the two.
        -- At 0.10: full braking above ~10 deg/s, full wheel holds a
        -- ~10 deg/s turn (360 in ~34 s). Lower for quicker turns that
        -- coast longer; raise if the hull still drifts after centering,
        -- but back off if it hunts around straight.
        yawDampGain = 0.10,

        -- Deg/s treated as already stopped. Keep just above gimbal noise.
        -- This is the residual creep the damper will not chase out, so
        -- it is a floor on how precisely a heading can be held: at 0.5
        -- the hull can still drift half a degree per second. Raise it
        -- if the props chatter around straight.
        yawDampDeadband = 0.5,

        -- Positive RSC RPM is backward on this hull; invert so +command is forward.
        invertLeft = true,
        invertRight = true,

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
        throttleLever = "throttle_lever_0",
    },

    -- ------------------------
    -- VNAV: vertical navigation (altitude hold + landing)
    -- ------------------------
    VNAV = {
        -- type: throttle_lever
        -- Lever 1-15 maps linearly onto the altitude range; detent 0 is land
        -- (fixed sink, then optical flare).
        burnerLever = "throttle_lever_1",

        -- type: Create rotational speed controller
        -- Drives all vertical propellers together (single shared RSC).
        -- Leftover +up boost when VerticalSpeedHold is short of desiredVS.
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
