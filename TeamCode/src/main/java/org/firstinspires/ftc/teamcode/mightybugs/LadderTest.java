package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Ladder Test", group = "Test")
public class LadderTest extends LinearOpMode {
    private DcMotor ladderLeft;
    private DcMotor ladderRight;

    private final double DRIVE_GAIN = 0.8;

    public void setup() {
        // Identify the servo.
        ladderLeft  = hardwareMap.get(DcMotor.class, "ladderLeft");
        ladderRight = hardwareMap.get(DcMotor.class, "ladderRight");

        // Initialize the directions.
        ladderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ladderRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // We do not want the ladder to fall down when not powered.
        ladderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ladderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoders.
        ladderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ladderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Restart the motors.
        ladderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ladderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left ladder", "Position %d", ladderLeft.getCurrentPosition());
            telemetry.addData("Right ladder","Position %d", ladderRight.getCurrentPosition());
            telemetry.update();

            double power = -gamepad1.left_stick_y * DRIVE_GAIN;

            ladderLeft.setPower(power);
            ladderRight.setPower(power);

            // Share the CPU.
            sleep(20);
        }

        ladderLeft.setPower(0.0);
        ladderRight.setPower(0.0);
    }
}
