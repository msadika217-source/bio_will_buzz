package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/*
 * Battery - reads battery voltage so motor power can be scaled to act the same
 * on a full battery (13+ V) or a tired one (12 V).
 *
 *   power you send = power you want x (NOMINAL_VOLTAGE / battery voltage)
 *
 * Not a TeleOp - it's a helper used by "zack code" and the tuners.
 */
public class Battery {

    private final VoltageSensor sensor;
    private double volts;

    public Battery(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        volts = Math.max(sensor.getVoltage(), 9.0);
    }

    /** Call once per loop. Smooths out the reading so power doesn't jitter. */
    public void update() {
        double v = sensor.getVoltage();
        if (v > 5.0) volts += 0.2 * (v - volts);   // ignore glitch readings
    }

    public double volts() { return volts; }

    /** Multiply motor power by this. Above 1.0 on a low battery, below 1.0 on a full one. */
    public double scale() { return AxonTeleOp.NOMINAL_VOLTAGE / Math.max(volts, 9.0); }
}