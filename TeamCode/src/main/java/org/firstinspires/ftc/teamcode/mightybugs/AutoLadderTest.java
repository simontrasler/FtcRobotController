package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Ladder Test", group = "Test")
public class AutoLadderTest extends OpModeBase {
    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        // Grip anything placed in the grabber.
        closeGrabber();

        // Make sure the swivel is in the correct position for driving.
        grabberSwivelToFloor();

        // Nudge the ladder up off the floor, so the second pixel will be
        // caught by the plow as we move forwards.
        nudgeLadder();

        // Make sure the camera is looking forward.
        cameraSwivelToCenter();

        // Get ready to drop the pixel.
        raiseLadder();

        if (opModeIsActive()) {
            // Rotate the grabber so we can place the pixel, and wait for the
            // swivel to stop moving.
            grabberSwivelToBackdrop();
            sleep(1000);

            // Release the pixel, and wait for the pixel to fall.
            openGrabber();
            sleep(500);
        }

        // Put the ladder back down.
        lowerLadder();
    }
}
