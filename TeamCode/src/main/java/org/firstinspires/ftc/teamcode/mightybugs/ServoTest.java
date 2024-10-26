package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {
    private Servo servo;

    public void setup() {
        // Identify the servo.
        servo = hardwareMap.get(Servo.class, "cameraSwivel");

        // Initialize the direction.
        servo.setDirection(Servo.Direction.REVERSE);

        // Initialize the range of motion. Note this is always calibrated on
        // the "forward" direction of the servo.
        servo.scaleRange(0.01, 0.21);
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servo.setPosition(Servo.MAX_POSITION);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(Servo.MIN_POSITION);
            }

            // Share the CPU.
            sleep(20);
        }
    }
}
