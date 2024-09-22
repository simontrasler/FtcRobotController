package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class AutonomousOpModeBase extends OpModeBase {
    /**
     * Constants for navigating to the spike marks.
     * The midpoints of the left and right spike marks are roughly 12 inches
     * to either side of the center line of the robot, and 6 inches closer
     * than the midpoint of the center spike mark, which is roughly 47 inches
     * away from the wall.
     */
    private enum SpikeMark { LEFT, CENTER, RIGHT };
    private final double INITIAL_MOVEMENT_X = 6.0;
    private final double SIDE_SPIKE_MARK_X = 41.0 - (ROBOT_LENGTH_INCHES / 2.0) - INITIAL_MOVEMENT_X;
    private final double SIDE_SPIKE_MARK_Y = 12.0;
    private final double SIDE_SPIKE_MARK_ANGLE = Math.atan2(SIDE_SPIKE_MARK_Y, SIDE_SPIKE_MARK_X);
    private final double WALL_BUFFER = 4.0;
    private final double TIMEOUT = 10.0;

    /**
     * Constants for navigating to the backstage.
     */
    private final double BACKSTAGE_BUFFER = 3.0;
    private final double BACKDROP_BUFFER = 10.0;
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

        // Make sure the swivel is in the correct position for driving.
        grabberSwivelToFloor();

        // Nudge the ladder up off the floor, so the second pixel will be
        // caught by the plow as we move forwards.
        nudgeLadder();

        // Make sure the camera is looking forward.
        cameraSwivelToCenter();

        // Go through the routine.
        setCameraZoom(2.0);
        SpikeMark spikeMark = findSpikeMark();
        setCameraZoom(1.0);
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

        while (opModeIsActive()) {
            List<Recognition> recognitions = getRecognitions();

            if (!recognitions.isEmpty()) {
                // If we have multiple recognitions, hope it's the first one!
                Recognition recognition = recognitions.get(0);
                double angle = recognition.estimateAngleToObject(AngleUnit.RADIANS);

                // We need to determine the randomization and remember it.
                if (angle < -SIDE_SPIKE_MARK_ANGLE / 2.0) {
                    spikeMark = SpikeMark.LEFT;
                } else if (angle > SIDE_SPIKE_MARK_ANGLE / 2.0) {
                    spikeMark = SpikeMark.RIGHT;
                }

                telemetry.addData("Spike Mark", "%s (angle %.2f)", spikeMark.toString(), angle);
                telemetry.update();

                break;
            } else if (elapsedTime.seconds() >= TIMEOUT) {
                // We don't have time to wait longer!
                break;
            }

            // Share the CPU.
            sleep(20);
        }

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
                List<Recognition> recognitions = getRecognitions();

                if (!recognitions.isEmpty()) {
                    // We found something, it can only be a pixel (we hope).
                    return spikeMark;
                }

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
        while (opModeIsActive() && timer.seconds() < 4.0) {
            List<Recognition> recognitions = getRecognitions();

            if (!recognitions.isEmpty()) {
                // We found something, it can only be a pixel (we hope).
                return SpikeMark.LEFT;
            }

            // Give the image recognition some time to work.
            sleep(100);
        }

        // Look right.
        cameraSwivelToRight();

        // Allow some time for a pixel to be detected.
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 4.0) {
            List<Recognition> recognitions = getRecognitions();

            if (!recognitions.isEmpty()) {
                // We found something, it can only be a pixel (we hope).
                return SpikeMark.RIGHT;
            }

            // Give the image recognition some time to work.
            sleep(100);
        }

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
