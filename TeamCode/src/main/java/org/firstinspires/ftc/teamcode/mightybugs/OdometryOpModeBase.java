package org.firstinspires.ftc.teamcode.mightybugs;

public abstract class OdometryOpModeBase extends OpModeBase {
    /**
     * Constants for converting odometry measurements to inches.
     */
    private final double TICKS_PER_INCH = 2000.0;

    /**
     * Odometry pods.
     */
    public void setup() {
        super.setup();
        setupOdometry();
    }

    private void setupOdometry() {

    }

    abstract public void runOpMode();
}
