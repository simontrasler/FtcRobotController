package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CameraTest extends LinearOpMode {
    private HuskyLens huskyLens;

    public void setup() {
        // Identify the camera.
        huskyLens = hardwareMap.get(HuskyLens.class, "camera");

        // Configure it to look for colors.
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        while (opModeIsActive()) {
            // Report any recognitions.
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", "%d", blocks.length);

            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Block", "id:%d x:%d y:%d size:%dx%d", block.id, block.x, block.y, block.width, block.height);
            }

            // Post the update to the Driver Hub.
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }
    }
}
