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
    },
    VNAV = {
        -- Optical AGL (metres) when the hull is sitting on the ground.
        -- Flare completes here; VNAV then latches landed and cuts heat.
        -- Measured on this ship: optical_sensor_0 reads 0.8 m at rest.
        touchdownAgl = 1.5,
    },
}

PERIPHERALS = {
    -- ------------------------
    -- LNAV: horizontal navigation (forward velocity + steering)
    -- ------------------------
    LNAV = {
        -- type: analog_transmission
        -- Drives the right propeller (Propeller:setPower sends 15 - power via setSignal).
        rightPropellerTransmission = "analog_transmission_9",

        -- type: analog_transmission
        -- Drives the left propeller (Propeller:setPower sends 15 - power via setSignal).
        leftPropellerTransmission = "analog_transmission_8",

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
        altitudeSensor = "altitude_sensor_1",

        -- type: hot_air_burner (list)
        -- Heat sources; BurnerBank fans the commanded amount out to every burner in this list.
        burners = {
            "hot_air_burner_2",
        },

        -- type: analog_transmission
        -- Drives all vertical propellers together (single shared transmission).
        -- Leftover +up boost when VerticalSpeedHold is short of desiredVS.
        verticalPropellerTransmission = "analog_transmission_10",

        -- type: optical_sensor (list)
        -- Downward sensors; worst-case (closest hasHit) AGL drives cruise
        -- terrain climb and the landing flare. Never wired to an actuator.
        opticalSensors = {
            "optical_sensor_0",
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
    },
}
