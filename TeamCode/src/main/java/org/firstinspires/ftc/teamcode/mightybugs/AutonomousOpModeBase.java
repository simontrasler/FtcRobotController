package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class AutonomousOpModeBase extends OpModeBase {
    /**
     * Constants for navigating during autonomous
     */
    private final double INITIAL_MOVEMENT_X = 6.0;
    private final double TIMEOUT = 10.0;

    /**
     * Constants for navigating
     * Field and Robot constants
     * */

    private final double TILE_LENGTH = 24.0;

    /**
     * Variables that define how we need to navigate the field.
     */
    private boolean isBlueAlliance;
    private boolean isNearBackstage;

    public AutonomousOpModeBase(boolean isBlueAlliance, boolean isNearBackstage) {
        this.isBlueAlliance  = isBlueAlliance;
        this.isNearBackstage = isNearBackstage;
    }

    @Override
    public void runOpMode() {
        setup();

        waitForStart();

        // Grip anything placed in the grabber.
        closeGrabber();

        /*
         * Insert function to raise the lifters to determined height
         * Insert function to adjust Vertical Servo
         * Insert function to adjust Horizontal Servos
         * */



        // Make sure the swivel is in the correct position for driving.
        grabberSwivelToFloor();

        // Make sure the camera is looking forward.
        cameraSwivelToCenter();

        // Go through the routine.
        SpikeMark spikeMark = findSpikeMark();
        placeSpikeMarkPixel(spikeMark);
        moveBackstage();
        findAprilTag(spikeMark);
        placeBackdropPixel();
        park();

        shutdown();
    }

    protected void setup() {
        super.setup();
        setupWheelsWithEncoders();
    }

    private SpikeMark findSpikeMark_attempt1() {
        SpikeMark spikeMark = SpikeMark.CENTER;

        // Move closer to the spike marks, for easier detection of the pixel.
        // This should be about 6 inches.
        moveForwards(INITIAL_MOVEMENT_X);

        // In case the camera does not detect the pixel, we will timeout and
        // assume it's on the center spike mark.
        ElapsedTime elapsedTime = new ElapsedTime();



        return spikeMark;
    }

    private SpikeMark findSpikeMark_attempt2() {
        // Move closer to the spike marks, for easier detection of the pixel.
        // This should be about 12 inches.
        moveForwards(INITIAL_MOVEMENT_X);

        // We have to be close to the pixel for the best chance of detecting
        // it. This means we can only look for one at a time. We only need to
        // consider two options, because if it's not one of them then we know
        // it's the third.
        ElapsedTime timer = new ElapsedTime();

        for (SpikeMark spikeMark : new SpikeMark[]{ SpikeMark.LEFT, SpikeMark.RIGHT }) {
            Point point = getSpikeMarkPoint(spikeMark);

            // Determine the target angle.
            double angle = Math.atan(point.y / point.x);

            // Turn.
            rotate(angle - getRobotAngle());

            // Allow some time for the pixel to be detected.
            timer.reset();
            while (opModeIsActive() && timer.seconds() < 4.0) {


                // Give the image recognition some time to work.
                sleep(100);
            }
        }

        // We did not find it, assume it is in the center.
        return SpikeMark.CENTER;
    }

    private SpikeMark findSpikeMark() {
        // Move closer to the spike marks, for easier detection of the pixel.
        // This should be about 12 inches.
        moveForwards(INITIAL_MOVEMENT_X);

        // We have to be close to the pixel for the best chance of detecting
        // it. This means we can only look for one at a time. We only need to
        // consider two options, because if it's not one of them then we know
        // it's the third.
        ElapsedTime timer = new ElapsedTime();

        // Look left.
        cameraSwivelToLeft();

        // Allow some time for a pixel to be detected.
        timer.reset();


        // Look right.
        cameraSwivelToRight();

        // Allow some time for a pixel to be detected.
        timer.reset();


        // We did not find it, assume it is in the center.
        return SpikeMark.CENTER;
    }

    private void placeSpikeMarkPixel(SpikeMark spikeMark) {
        // Find out where we are going.
        Point point = getSpikeMarkPoint(spikeMark);

        // Turn.
        double angle = Math.atan(point.y / point.x);
        rotate(angle - getRobotAngle());

        // Put the camera back in the center. Do this after the turn so we
        // don't jolt the IMU and get the wrong angle.
        cameraSwivelToCenter();

        // Move to place the pixel.
        double d = Math.sqrt(point.y * point.y + point.x * point.x) + INTAKE_DEPTH_INCHES - (ROBOT_LENGTH_INCHES / 2.0);
        moveForwards(d);

        // Reverse these steps, leaving the pixel behind.
        moveForwards(-d);

        // Return to the original starting orientation.
        rotate(-getRobotAngle());

        // Move back to the wall, leaving a little buffer.
        moveForwards(-INITIAL_MOVEMENT_X + WALL_BUFFER);
    }

    private void moveBackstage_attempt1() {
        // Rotate to have the camera face the backdrop.
        double angle = Math.PI / 3.0;

        if (isBlueAlliance) {
            angle = -angle;
        }

        rotate(angle);

        int direction;

        if (isBlueAlliance) {
            direction = 1;
        } else {
            direction = -1;
        }

        if (isNearBackstage) {
            moveForwards(TILE_LENGTH * direction);
        } else {
            // Robots not near the backstage have to move closer.
            moveForwards(TILE_LENGTH * direction * 3.0);
        }
    }

    private void moveBackstage() {
        int direction;

        if (isBlueAlliance) {
            direction = -1;
        } else {
            direction = 1;
        }

        if (isNearBackstage) {
            moveSideways(TILE_LENGTH * direction);
        } else {
            // Robots not near the backstage have to move closer.
            moveSideways(TILE_LENGTH * direction * 3.0);
        }
    }

    private void findAprilTag_attempt1(SpikeMark spikeMark) {
        setupWheelsWithoutEncoders();

        // Use low exposure time on the camera to reduce motion blur.
        setManualExposure(6, 250);

        // Blue Alliance has AprilTags 1, 2, 3; Red Alliance has 4, 5, 6.
        int centerId = (isBlueAlliance) ? 2 : 5;

        while (opModeIsActive()) {
            // Check for detections.
            List<AprilTagDetection> detections = getDetections();
            AprilTagDetection foundDetection = null;

            populateTelemetry();
            telemetry.update();

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    if ((detection.id == centerId - 1 && spikeMark == SpikeMark.LEFT) ||
                            (detection.id == centerId && spikeMark == SpikeMark.CENTER) ||
                            (detection.id == centerId + 1 && spikeMark == SpikeMark.RIGHT)) {
                        foundDetection = detection;
                        break;
                    }
                }
            }

            if (foundDetection != null) {
                double drive  = foundDetection.ftcPose.range - BACKSTAGE_BUFFER;
                double strafe = foundDetection.ftcPose.yaw;
                double turn   = foundDetection.ftcPose.bearing;

                final double AUTO_SPEED_GAIN  = 0.02;
                final double AUTO_STRAFE_GAIN = 0.015;
                final double AUTO_TURN_GAIN   = 0.01;

                if (Math.abs(drive) < BACKSTAGE_BUFFER) {
                    // Stop the robot, and go to the next step.
                    setWheelPower(0.0, 0.0, 0.0, 0.0);
                    break;
                } else {
                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive  = Range.clip(drive * AUTO_SPEED_GAIN, -DRIVE_GAIN, DRIVE_GAIN);
                    strafe = Range.clip(strafe * AUTO_STRAFE_GAIN, -DRIVE_GAIN, DRIVE_GAIN);
                    turn   = Range.clip(turn * AUTO_TURN_GAIN, -TURN_GAIN, TURN_GAIN);

                    // Move towards the AprilTag.
                    moveByDirection(false, drive, strafe, turn);
                }
            }

            // Share the CPU.
            sleep(20);
        }

        sleep(250);
        setupWheelsWithEncoders();
    }

    private void findAprilTag(SpikeMark spikeMark) {
        // Move forwards.
        moveForwards(TILE_LENGTH);

        // Rotate to have the camera face the backdrop.
        double angle = Math.PI / 2.0;

        if (isBlueAlliance) {
            angle = -angle;
        }

        rotate(angle);

        // Move closer.
        moveForwards(TILE_LENGTH - BACKDROP_BUFFER);
    }

    private void placeBackdropPixel() {
        // Rotate to have the ladder face the backdrop. The robot's current
        // angle is relative to its starting position. The desired angle is
        // 90 degrees to that, facing away from the backdrop.
        double currentAngle = getRobotAngle();
        double turnAngle;

        if (isBlueAlliance) {
            // Current angle is expected to be negative.
            turnAngle = Math.PI / 2.0 - currentAngle;
        } else {
            // Current angle is expected to be positive.
            turnAngle = -Math.PI / 2.0 - currentAngle;
        }

        rotate(turnAngle);

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

        lowerLadder();
    }

    private void park() {
        // Expect the robots closest to the backstage to park in the corners,
        // so the robots coming from farthest away just stop after placing
        // their pixel. Remember at this point the robot is facing away from
        // the backdrop, so on the Blue Alliance side the corner is to its
        // right.
        if (isNearBackstage) {
            int direction;

            if (isBlueAlliance) {
                direction = 1;
            } else {
                direction = -1;
            }

            moveSideways(TILE_LENGTH * direction);
        }
    }

    private Point getSpikeMarkPoint(SpikeMark spikeMark) {
        double x, y;

        switch (spikeMark) {
            case LEFT:
                x = SIDE_SPIKE_MARK_X;
                y = -SIDE_SPIKE_MARK_Y;
                break;
            case RIGHT:
                x = SIDE_SPIKE_MARK_X;
                y = SIDE_SPIKE_MARK_Y;
                break;
            default:
                x = SIDE_SPIKE_MARK_X + 6.0;
                y = 0.0;
                break;
        }

        return new Point(x, y);
    }

    private class Point {
        public double x;
        public double y;

        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
