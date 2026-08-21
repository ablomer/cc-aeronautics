-- Ship-specific peripheral configuration.
--
-- This is the single source of truth for which physical peripheral (by its
-- ComputerCraft string ID) backs each logical role in the code. Other files
-- should reference PERIPHERALS.<SYSTEM>.<role>.id instead of hardcoding
-- peripheral name strings, so that re-wiring the ship only requires editing
-- this file.
--
-- Each entry is a table: { id = "<string id>", type = "<peripheral type>", description = "..." }
-- Entries that represent a bank of identical peripherals (e.g. multiple
-- burners) are lists of such tables.

PERIPHERALS = {
    -- ------------------------
    -- LNAV: horizontal navigation (forward velocity + steering)
    -- ------------------------
    LNAV = {
        rightPropellerTransmission = {
            id = "analog_transmission_8",
            type = "analog_transmission",
            description = "Drives the right propeller (Propeller:setPower sends 15 - power via setSignal).",
        },
        leftPropellerTransmission = {
            id = "analog_transmission_9",
            type = "analog_transmission",
            description = "Drives the left propeller (Propeller:setPower sends 15 - power via setSignal).",
        },
        throttleLever = {
            id = "throttle_lever_7",
            type = "throttle_lever",
            description = "Manual throttle input; also the velocity target source captured when engaging velocity hold.",
        },
        velocitySensor = {
            id = "velocity_sensor_3",
            type = "velocity_sensor",
            description = "Reports current ship velocity, used by VelocityHold and shown on the display.",
        },
        steeringWheel = {
            id = "steering_wheel_3",
            type = "steering_wheel",
            description = "Manual steering input, read whenever nav (bearing hold) steering is not engaged.",
        },
        navigationTable = {
            id = "navigation_table_1",
            type = "navigation_table",
            description = "Provides nav target bearing/heading/distance; drives BearingHold autopilot steering.",
        },
    },

    -- ------------------------
    -- VNAV: vertical navigation (altitude hold)
    -- ------------------------
    VNAV = {
        burnerLever = {
            id = "throttle_lever_8",
            type = "throttle_lever",
            description = "Sets target altitude; lever position (0-15) is mapped linearly onto the altitude range.",
        },
        altitudeSensor = {
            id = "altitude_sensor_1",
            type = "altitude_sensor",
            description = "Reports current height and vertical speed, consumed by AltitudeHold's cascade loops.",
        },
        burners = {
            {
                id = "hot_air_burner_2",
                type = "hot_air_burner",
                description = "Heat source; BurnerBank fans the commanded amount out to every burner in this list.",
            },
        },
        verticalPropellerTransmission = {
            id = "analog_transmission_10",
            type = "analog_transmission",
            description = "Drives all vertical propellers together (single shared transmission).",
        },
        opticalSensors = {
            {
                id = "optical_sensor_0",
                type = "optical_sensor",
                description = "Declared for future vertical obstacle/ground sensing; not yet read by the control loop.",
            },
        },
    },

    -- ------------------------
    -- DEBUG: peripherals only exercised by debug.lua's standalone menu
    -- ------------------------
    DEBUG = {
        laserSensor = {
            id = "laser_pointer_0",
            type = "laser_pointer",
            description = "Introspected live via debug.lua's 'Laser sensor' menu entry.",
        },
        opticalSensor = {
            id = "optical_sensor_0",
            type = "optical_sensor",
            description = "Same physical peripheral as VNAV.opticalSensors[1]; introspected via debug.lua's 'Optical sensor' menu entry.",
        },
    },
}
