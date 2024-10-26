package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Human", group = "Human")
public class HumanOpMode extends OpModeBase {
    protected void setup() {
        super.setup();
        setupWheelsWithoutEncoders();
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        // Open the grabber ready for the next pixel.
        openGrabber();

        // Make sure the swivel is in the correct position for driving.
        grabberSwivelToFloor();

        // Make sure the camera is looking forward.
        cameraSwivelToCenter();

        while (opModeIsActive()) {
            // Scan for objects, and publish telemetry.
            populateTelemetry();

            // Drive.
            double drive  = 0.0;
            double strafe = 0.0;
            double turn   = 0.0;

            // If the dpad is being used, ignore the joystick.
            if (gamepad1.dpad_up) {
                drive = DRIVE_GAIN;
            } else if (gamepad1.dpad_left) {
                strafe = -DRIVE_GAIN;
            } else if (gamepad1.dpad_right) {
                strafe = DRIVE_GAIN;
            } else if (gamepad1.dpad_down) {
                drive = -DRIVE_GAIN;
            } else {
                // Drive using the joystick.
                drive = -gamepad1.left_stick_y * DRIVE_GAIN;
                strafe = gamepad1.left_stick_x * DRIVE_GAIN;
                turn = -gamepad1.right_stick_x * TURN_GAIN;
            }
            telemetry.addData("Wheels","Drive %5.2f, Strafe %5.2f, Turn %5.2f", drive, strafe, turn);

            // Operate the grabber.
            if (gamepad2.left_bumper) {
                openGrabber();
            } else if (gamepad2.right_bumper) {
                closeGrabber();
            }

            // Operate the ladder.
            if (gamepad2.y) {
                raiseLadder();
            } else if (gamepad2.a) {
                lowerLadder();
            } else {
                double power = -gamepad2.left_stick_y * LADDER_GAIN;
                moveLadderByDirection(power);
            }

            // Operate the swivel.
            if (gamepad2.x) {
                grabberSwivelToBackdrop();
            } else if (gamepad2.b) {
                grabberSwivelToFloor();
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Push human control to the motors.
            moveByDirection(true, drive, strafe, turn);

            // Share the CPU.
            sleep(20);
        }

        shutdown();
    }
}
