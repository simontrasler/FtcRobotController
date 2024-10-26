package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Test", group = "Test")
public class EncoderTest extends OpModeBase {
    private final double TEST_DISTANCE = 12.0;

    protected void setup() {
        super.setup();
        setupWheelsWithoutEncoders();
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                moveForwards(TEST_DISTANCE);
            } else if (gamepad1.dpad_down) {
                moveForwards(-TEST_DISTANCE);
            } else if (gamepad1.dpad_left) {
                moveSideways(-TEST_DISTANCE);
            } else if (gamepad1.dpad_right) {
                moveSideways(TEST_DISTANCE);
            } else if (gamepad1.a) {
                rotate(Math.PI * 2.0);
            }

            // Share the CPU.
            sleep(20);
        }

        shutdown();
    }
}
