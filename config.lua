-- Ship-specific peripheral configuration.
--
-- This is the single source of truth for which physical peripheral (by its
-- ComputerCraft string ID) backs each logical role in the code. Other files
-- should reference PERIPHERALS.<SYSTEM>.<role> instead of hardcoding
-- peripheral name strings, so that re-wiring the ship only requires editing
-- this file.
--
-- Each role is documented with a comment giving its peripheral type and a
-- short description, immediately above the string ID.

PERIPHERALS = {
    -- ------------------------
    -- LNAV: horizontal navigation (forward velocity + steering)
    -- ------------------------
    LNAV = {
        -- type: analog_transmission
        -- Drives the right propeller (Propeller:setPower sends 15 - power via setSignal).
        rightPropellerTransmission = "analog_transmission_8",

        -- type: analog_transmission
        -- Drives the left propeller (Propeller:setPower sends 15 - power via setSignal).
        leftPropellerTransmission = "analog_transmission_9",

        -- type: throttle_lever
        -- Manual throttle input; also the velocity target source captured when engaging velocity hold.
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
    -- VNAV: vertical navigation (altitude hold)
    -- ------------------------
    VNAV = {
        -- type: throttle_lever
        -- Sets target altitude; lever position (0-15) is mapped linearly onto the altitude range.
        burnerLever = "throttle_lever_8",

        -- type: altitude_sensor
        -- Reports current height and vertical speed, consumed by AltitudeHold's cascade loops.
        altitudeSensor = "altitude_sensor_1",

        -- type: hot_air_burner (list)
        -- Heat sources; BurnerBank fans the commanded amount out to every burner in this list.
        burners = {
            "hot_air_burner_2",
        },

        -- type: analog_transmission
        -- Drives all vertical propellers together (single shared transmission).
        verticalPropellerTransmission = "analog_transmission_10",

        -- type: optical_sensor (list)
        -- Declared for future vertical obstacle/ground sensing; not yet read by the control loop.
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
        laserSensor = "laser_pointer_0",

        -- type: optical_sensor
        -- Same physical peripheral as VNAV.opticalSensors[1]; introspected via debug.lua's 'Optical sensor' menu entry.
        opticalSensor = "optical_sensor_0",
    },
}
