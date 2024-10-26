package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Motor Test (Sync)", group = "Test")
public class MotorSyncTest extends LinearOpMode {
    private DcMotor motor;

    /**
     * Constants for encoder driving.
     */
    private final double COUNTS_PER_MOTOR_REV  = 537.6;
    private final double WHEEL_DIAMETER_INCHES = 1.375;
    private final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / WHEEL_DIAMETER_INCHES / Math.PI;

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

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                move(6.0, 5.0);
            } else if (gamepad1.dpad_down) {
                move(-6.0, 5.0);
            }

            // Share the CPU.
            sleep(20);
        }

        motor.setPower(0.0);
    }

    private void move(double distance, double timeout) {
        ElapsedTime timer = new ElapsedTime();

        // Convert distance to motor ticks.
        int ticks = (int)(distance * COUNTS_PER_INCH);
        int target = motor.getCurrentPosition() + ticks;

        // The power will depend on the direction of travel.
        double power = 0.5 * Math.signum(distance);

        // Get the motor ready.
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // We are ready, so reset the timer and start motion.
        timer.reset();
        motor.setPower(power);

        while (opModeIsActive() && motor.isBusy() && timer.seconds() < timeout) {
            // Report the current state of the motor.
            telemetry.addData("Motor", "Target %d, Position %d", target, motor.getCurrentPosition());
            telemetry.update();
        }

        // The objective is achieved, reset the state for the next move.
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
