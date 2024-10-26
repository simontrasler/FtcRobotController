package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Motor Test (Async)", group = "Test")
public class MotorAsyncTest extends LinearOpMode {
    private DcMotor motor;

    private ElapsedTime timer = new ElapsedTime();

    /**
     * Constants for encoder driving.
     */
    private final double COUNTS_PER_MOTOR_REV  = 537.6;
    private final double WHEEL_DIAMETER_INCHES = 1.375;
    private final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / WHEEL_DIAMETER_INCHES / Math.PI;
    private final double TIMEOUT = 5.0;

    public void setup() {
        // Identify the servo.
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Initialize the direction.
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // We do not want the ladder to fall down when not powered.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the current encoder position to zero.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motors on, ready to track distance traveled.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        boolean isMoving = false;
        int target = 0;

        while (opModeIsActive()) {
            // Only check for new gamepad input if we were not already moving.
            if (isMoving) {
                if (motor.isBusy() && timer.seconds() < TIMEOUT) {
                    // Report the current state of the motor.
                    telemetry.addData("Motor", "Target %d, Position %d", target, motor.getCurrentPosition());
                    telemetry.update();
                } else {
                    stopMotor();
                    isMoving = false;
                }
            } else if (gamepad1.dpad_up) {
                target = startMotor(6.0);
                isMoving = true;
            } else if (gamepad1.dpad_down) {
                target = startMotor(-6.0);
                isMoving = true;
            }

            // Share the CPU.
            sleep(20);
        }

        stopMotor();
    }

    private int startMotor(double distance) {
        // Convert distance to motor ticks.
        int ticks = (int) (distance * COUNTS_PER_INCH);
        int target = motor.getCurrentPosition() + ticks;

        // The power will depend on the direction of travel.
        double power = 0.5 * Math.signum(distance);

        // Get the motor ready.
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // We are ready, so reset the timer and start motion.
        timer.reset();
        motor.setPower(power);

        return target;
    }

    private void stopMotor() {
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
