package org.firstinspires.ftc.teamcode.mightybugs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class OpModeBase extends LinearOpMode {
    /**
     * Constants for the TensorFlow model.
     */
    private final String TFOD_MODEL_FILE = "CenterStage.tflite";
    private final String[] TFOD_MODEL_LABELS = { "Pixel" };

    /**
     * Constants for controllability.
     * Smaller values are "slower", and 1.0 is "normal".
     */
    protected final double DRIVE_GAIN = 0.5;
    protected final double TURN_GAIN  = 0.3;
    protected final double LADDER_GAIN = 0.5;

    /**
     * Dimensions of the robot.
     */
    protected final double ROBOT_LENGTH_INCHES = 13.85;
    protected final double INTAKE_DEPTH_INCHES = 10.0;

    /**
     * Constants for encoder driving.
     */
    private final double TICKS_PER_MOTOR_REV = 537.6;
    private final double TIMEOUT = 10.0;

    /**
     * Constants for wheels operation.
     */
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    private final double WHEEL_TICKS_PER_INCH = TICKS_PER_MOTOR_REV / WHEEL_DIAMETER_INCHES / Math.PI;
    private final double FORWARDS_FUDGE_FACTOR = 12.0 / 11.0;
    private final double SIDEWAYS_FUDGE_FACTOR = 12.0 / 13.0;

    /**
     * Constants for ladder operation.
     */
    private final double SPOOL_DIAMETER_INCHES = 1.375;
    private final double SPOOL_TICKS_PER_INCH = TICKS_PER_MOTOR_REV / SPOOL_DIAMETER_INCHES / Math.PI;
    private final double LADDER_FUDGE_FACTOR = 12.0 / 12.5;
    private final double LADDER_MIN_HEIGHT_INCHES = 0.0;
    private final double LADDER_STEP_HEIGHT_INCHES = 2.0;
    private final double LADDER_SWIVEL_FLOOR_INCHES = 4.0;
    private final double LADDER_MAX_HEIGHT_INCHES = 28.0;

    /**
     * Objects for input/output.
     */
    private OpenCvWebcam webcam;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private IMU imu;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor ladderLeft;
    private DcMotor ladderRight;
    private DcMotor swivelLeft;
    private DcMotor swivelRight;
    private Servo grabberLeft;
    private Servo grabberRight;
    private Servo grabberSwivel;
    private Servo cameraSwivel;

    /**
     * Setup functions.
     */

    protected void setup() {
        // Do the setup.
        setupImu();
        setupMotors();
        setupServos();

        // Acknowledge that the op mode is initialized.
        telemetry.addData("Mode", "%s ready", this.getClass().getSimpleName());
        telemetry.update();
    }

    private void setupMotors() {
        setupWheelMotors();
        setupLadderMotors();
    }

    private void setupImu() {
        // Identify the IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize the IMU.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    private void setupWheelMotors() {
        // Identify the motors.
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Initialize directions. The "forward" direction is clockwise,
        // looking from the motor body so the motors on the right side must run
        // in reverse.
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    protected void setupWheelsWithoutEncoders() {
        // The motors will run at the speed they are given.
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void setupWheelsWithEncoders() {
        // Stop the motors and set their current encoder positions to zero.
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Re-start the encoders.
        resetWheelsState();
    }

    protected void resetWheelsState() {
        // Turn the motors on, ready to track distance traveled.
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void prepareWheelsForDistance() {
        // Turn the motors on, to travel the distance given.
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setupLadderMotors() {
        // Identify the motors.
        ladderLeft = hardwareMap.get(DcMotor.class, "ladderLeft");
        ladderRight = hardwareMap.get(DcMotor.class, "ladderRight");

        // Initialize the directions.
        ladderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ladderRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // We do not want the ladder to fall down when not powered.
        ladderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ladderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the current encoder position to zero.
        ladderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ladderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motors on, by default ready for human control.
        ladderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ladderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setupSwivelMotors() {
        // Identify the motors.
        swivelLeft = hardwareMap.get(DcMotor.class, "swivelLeft");
        swivelRight = hardwareMap.get(DcMotor.class, "swivelRight");

        // Initialize the directions.
        swivelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        swivelRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // We do not want the ladder to fall down when not powered.
        swivelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swivelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the current encoder position to zero.
        swivelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the motors on, by default ready for human control.
        swivelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swivelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setupServos() {
        setupGrabberServos();
        setupSwivelServos();
    }

    private void setupGrabberServos() {
        // Identify the grabber servos. Do not set the starting positions, so
        // the team can position them as needed.
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");

        // Initialize directions.
        grabberLeft.setDirection(Servo.Direction.FORWARD);
        grabberRight.setDirection(Servo.Direction.REVERSE);

        // Initialize the range of motion. Note this is always calibrated on
        // the "forward" direction of the servo.
        grabberLeft.scaleRange(0.01, 0.21);
        grabberRight.scaleRange(0.62, 0.82);
    }

    private void setupSwivelServos() {
        // Identify the swivel servo. Do not set the starting positions, so
        // the team can position them as needed.
        grabberSwivel = hardwareMap.get(Servo.class, "grabberSwivel");
        cameraSwivel  = hardwareMap.get(Servo.class, "cameraSwivel");

        // Initialize direction.
        grabberSwivel.setDirection(Servo.Direction.REVERSE);
        cameraSwivel.setDirection(Servo.Direction.FORWARD);

        // Initialize the range of motion. Note this is always calibrated on
        // the "forward" direction of the servo.
        grabberSwivel.scaleRange(0.28, 0.68);
        cameraSwivel.scaleRange(0.35, 0.6);
    }

    /**
     * Shutdown functions.
     */

    protected void shutdown() {
        // Stop the wheels.
        setWheelPower(0.0, 0.0, 0.0, 0.0);

        // Adjust the swivel so the grabber can descend safely.
        grabberSwivelToFloor();

        // Stop the ladder.
        ladderLeft.setPower(0.0);
        ladderRight.setPower(0.0);

        // Save CPU resources when camera is no longer needed.
        visionPortal.close();

        // Acknowledge that the op mode is completed.
        telemetry.addData("Mode", "%s finished", this.getClass().getSimpleName());
        telemetry.update();
    }

    /**
     * Telemetry functions.
     */

    protected void populateTelemetry() {
        populateTelemetryVisionPortal();
        populateTelemetryNewLine();

        populateTelemetryServos();
        populateTelemetryNewLine();
    }

    private void populateTelemetryVisionPortal() {
        populateTelemetryNewLine();

        populateTelemetryAprilTag();
        populateTelemetryNewLine();
    }


    private void populateTelemetryMotion() {
        telemetry.addData("Motion", " ");
        telemetry.addData("- Back left",   "Now %d, Target %d", backLeft.getCurrentPosition(),   backLeft.getTargetPosition());
        telemetry.addData("- Back right",  "Now %d, Target %d", backRight.getCurrentPosition(),  backRight.getTargetPosition());
        telemetry.addData("- Front left",  "Now %d, Target %d", frontLeft.getCurrentPosition(),  frontLeft.getTargetPosition());
        telemetry.addData("- Front right", "Now %d, Target %d", frontRight.getCurrentPosition(), frontRight.getTargetPosition());
    }

    private void populateTelemetryLadder() {
        telemetry.addData("Ladder", " ");
        telemetry.addData("- Left",   "Now %d, Target %d", ladderLeft.getCurrentPosition(),  ladderLeft.getTargetPosition());
        telemetry.addData("- Right",  "Now %d, Target %d", ladderRight.getCurrentPosition(), ladderRight.getTargetPosition());
    }

    protected List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    private void populateTelemetryAprilTag() {
        List<AprilTagDetection> currentDetections = getDetections();
        telemetry.addData("# Tags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            populateTelemetryNewLine();

            // Presence of metadata guarantees positional information.
            if (detection.metadata != null) {
                telemetry.addData("Found", "Tag ID %d", detection.id);
                telemetry.addData("- Center", "%.0f / %.0f", detection.center.x, detection.center.y);
                telemetry.addData("- Range", "%.0f", detection.ftcPose.range);
                telemetry.addData("- Angle", "%.0f", detection.ftcPose.bearing);
            } else {
                telemetry.addData("Unknown", "Tag ID %d", detection.id);
            }
        }
    }

    private void populateTelemetryServos() {
        telemetry.addData("Grabber", "Left %.2f, Right %.2f", grabberLeft.getPosition(), grabberRight.getPosition());
        telemetry.addData("Grabber swivel", "At %.2f", grabberSwivel.getPosition());
        telemetry.addData("Camera swivel", "At %.2f", cameraSwivel.getPosition());
    }

    private void populateTelemetryNewLine() {
        telemetry.addData(""," ");
    }

    /**
     * Motion functions.
     */

    protected void moveForwards(double distance) {
        // To move forwards, we want the left motors to rotate clockwise, and
        // the right motors to rotate anti-clockwise.
        moveByDistance(DRIVE_GAIN, distance * FORWARDS_FUDGE_FACTOR, true, true, false, false, TIMEOUT);
    }

    protected void moveSideways(double distance) {
        // To move to the right, we want the front motors to rotate clockwise,
        // and the back motors to rotate anti-clockwise.
        moveByDistance(DRIVE_GAIN, distance * SIDEWAYS_FUDGE_FACTOR, false, true, false, true, TIMEOUT);
    }

    protected void rotate(double angle) {
        if (angle != 0.0) {
            // To turn to the right, we want all the motors to rotate clockwise.
            // The wheels will travel along the circumference of a circle whose
            // diameter is the robot length. Note, the wheels themselves must
            // travel *twice* the radius times the angle in radians.
            double distance = angle * ROBOT_LENGTH_INCHES;
            moveByDistance(TURN_GAIN, distance, true, true, true, true, TIMEOUT);
        }
    }

    /**
     * Move the robot a set distance (using encoders). The motor directions are
     * used to determine whether the robot moves forwards, sideways, or turns.
     *
     * @param gain The speed to use
     * @param inches The distance to move
     * @param backLeftClockwise True if the back left motor should turn clockwise, false otherwise
     * @param frontLeftClockwise True if the front left motor should turn clockwise, false otherwise
     * @param backRightClockwise True if the back right motor should turn clockwise, false otherwise
     * @param frontRightClockwise True if the front right motor should turn clockwise, false otherwise
     * @param timeout The maximum time to drive, for safety reasons
     */
    private void moveByDistance(double gain, double inches, boolean backLeftClockwise, boolean frontLeftClockwise, boolean backRightClockwise, boolean frontRightClockwise, double timeout) {
        ElapsedTime elapsedTime = new ElapsedTime();

        // Only move if still active.
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller.
            int distance = (int) (inches * WHEEL_TICKS_PER_INCH);

            int backLeftStart   = backLeft.getCurrentPosition();
            int frontLeftStart  = frontLeft.getCurrentPosition();
            int backRightStart  = backRight.getCurrentPosition();
            int frontRightStart = frontRight.getCurrentPosition();

            // Although the right-side motors are set to run in reverse, the
            // position counters still count in the original direction.
            int backLeftTarget   = backLeftStart   + (backLeftClockwise   ? 1 : -1) * distance;
            int frontLeftTarget  = frontLeftStart  + (frontLeftClockwise  ? 1 : -1) * distance;
            int backRightTarget  = backRightStart  - (backRightClockwise  ? 1 : -1) * distance;
            int frontRightTarget = frontRightStart - (frontRightClockwise ? 1 : -1) * distance;

            backLeft.setTargetPosition(backLeftTarget);
            frontLeft.setTargetPosition(frontLeftTarget);
            backRight.setTargetPosition(backRightTarget);
            frontRight.setTargetPosition(frontRightTarget);

            prepareWheelsForDistance();

            // We are ready, so reset the timer and start motion.
            elapsedTime.reset();
            setWheelPower(gain, gain, gain, gain);

            while (opModeIsActive() && (elapsedTime.seconds() < timeout) && backLeft.isBusy() && frontLeft.isBusy() && backRight.isBusy() && frontRight.isBusy()) {
                // Keep looping while we wait.
                populateTelemetryMotion();
                telemetry.update();

                // Share the CPU.
                sleep(5);
            }

            // Stop.
            setWheelPower(0.0, 0.0, 0.0, 0.0);
            resetWheelsState();

            // Short pause to allow the robot to come to rest.
            sleep(100);
        }
    }

    /**
     * Move the robot in a set direction (not using encoders).
     *
     * @param isHumanPov True if the other inputs are to be read from the human
     *                   driver's point of view, false if from the robot's own
     *                   point of view.
     * @param drive The amount to move forwards
     * @param strafe The amount to move to the right
     * @param yaw The amount to turn
     */
    protected void moveByDirection(boolean isHumanPov, double drive, double strafe, double yaw) {
        if (isHumanPov) {
            // Calculate the angle of the robot relative to its starting position,
            // so we can move the robot in the direction desired from the point
            // of view of the human driver. We measure angles clockwise from the
            // front of the robot, looking down on it.
            double gamepadAngle = Math.atan2(strafe, drive);
            double robotAngle = getRobotAngle();

            double compensatingAngle = gamepadAngle - robotAngle;

            // Adjust the directions of travel accordingly.
            double speed = Math.sqrt(drive * drive + strafe * strafe);
            drive  = Math.cos(compensatingAngle) * speed;
            strafe = Math.sin(compensatingAngle) * speed;
        }

        // Calculate wheel powers.
        double frontLeftPower  = drive + strafe - yaw;
        double frontRightPower = drive - strafe + yaw;
        double backLeftPower   = drive - strafe - yaw;
        double backRightPower  = drive + strafe + yaw;

        // Get the strongest of all the wheel powers...
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        // ...and normalize them all to be 1.0 or less.
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send power to the wheels.
        setWheelPower(backLeftPower, frontLeftPower, backRightPower, frontRightPower);
    }

    protected double getRobotAngle() {
        // We are measuring angles in the opposite direction to the standard
        // right-hand rule, so invert the sign.
        return -imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    protected void setWheelPower(double backLeftPower, double frontLeftPower, double backRightPower, double frontRightPower) {
        backLeft.setPower(backLeftPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        frontRight.setPower(frontRightPower);
    }

    /**
     * Grabber functions.
     */

    protected void openGrabber() {
        grabberLeft.setPosition(Servo.MAX_POSITION);
        grabberRight.setPosition(Servo.MAX_POSITION);
    }

    protected void closeGrabber() {
        grabberLeft.setPosition(Servo.MIN_POSITION);
        grabberRight.setPosition(Servo.MIN_POSITION);
    }

    /**
     * Swivel functions.
     */

    protected void grabberSwivelToFloor() {
        grabberSwivel.setPosition(Servo.MIN_POSITION);
    }

    protected void grabberSwivelToBackdrop() {
        if (ladderRight.getCurrentPosition() < LADDER_SWIVEL_FLOOR_INCHES * SPOOL_TICKS_PER_INCH * LADDER_FUDGE_FACTOR) {
            // The ladder is in the lowered position. The swivel would catch
            // on the chassis, so do not allow the movement.
            return;
        }
        grabberSwivel.setPosition(Servo.MAX_POSITION);
    }

    protected void cameraSwivelToLeft() {
        cameraSwivel.setPosition(Servo.MIN_POSITION);
    }

    protected void cameraSwivelToCenter() {
        cameraSwivel.setPosition(0.5);
    }

    protected void cameraSwivelToRight() {
        cameraSwivel.setPosition(Servo.MAX_POSITION);
    }

    /**
     * Ladder functions.
     */

    /**
     * Move the ladder to set positions.
     *
     * @param targetPosition The target height
     * @param timeout The maximum amount of time to move, for safety reasons
     */
    protected void moveLadderByDistance(double targetPosition, double timeout) {
        ElapsedTime elapsedTime = new ElapsedTime();

        // Stop the ladder, so we can measure its current position.
        ladderLeft.setPower(0.0);
        ladderRight.setPower(0.0);

        // Move to set positions, not relative to where we are starting from.
        int target = (int)(targetPosition * SPOOL_TICKS_PER_INCH * LADDER_FUDGE_FACTOR);

        if (target < ladderRight.getCurrentPosition()) {
            // We are lowering the ladder, so make sure the swivel is in the
            // correct position. The ladders are locked together, so we can
            // take the current position from either one.
            grabberSwivelToFloor();
        }

        ladderLeft.setTargetPosition(target);
        ladderRight.setTargetPosition(target);

        ladderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ladderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // We are ready, so reset the timer and start motion.
        elapsedTime.reset();
        ladderLeft.setPower(LADDER_GAIN);
        ladderRight.setPower(LADDER_GAIN);

        while (opModeIsActive() && (elapsedTime.seconds() < timeout) && ladderLeft.isBusy() && ladderRight.isBusy()) {
            // Keep looping while we wait.
            populateTelemetryLadder();
            telemetry.update();

            // Share the CPU.
            sleep(5);
        }

        ladderLeft.setPower(0.0);
        ladderRight.setPower(0.0);

        ladderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ladderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Ladder", "Left %d, Right %d", ladderLeft.getCurrentPosition(), ladderRight.getCurrentPosition());
    }

    /**
     * Move the ladder in a set direction (not using encoders).
     *
     * @param power The speed to use.
     */
    protected void moveLadderByDirection(double power) {
        telemetry.addData("Ladder", "Left %d, Right %d", ladderLeft.getCurrentPosition(), ladderRight.getCurrentPosition());

        double inches = ladderRight.getCurrentPosition() / SPOOL_TICKS_PER_INCH / LADDER_FUDGE_FACTOR;

        if (inches >= LADDER_MAX_HEIGHT_INCHES) {
            // Do not allow the ladder to go beyond the maximum height.
            if (power > 0.0) {
                power = 0.0;
            }
        } else if (power < 0.0) {
            // We are descending. Reduce power as the ladder approaches the
            // minimum position.
            power = Math.min(power, inches / LADDER_SWIVEL_FLOOR_INCHES);
        }

        ladderLeft.setPower(power);
        ladderRight.setPower(power);
    }

    protected void lowerLadder() {
        moveLadderByDistance(LADDER_MIN_HEIGHT_INCHES, TIMEOUT);
    }

    protected void nudgeLadder() {
        moveLadderByDistance(LADDER_STEP_HEIGHT_INCHES, TIMEOUT);
    }

    protected void raiseLadder() {
        moveLadderByDistance(LADDER_MAX_HEIGHT_INCHES, TIMEOUT);
    }

    /**
     * Camera functions.
     */

    protected void setManualExposure(int exposureMillis, int gain) {
        // Make sure camera is streaming before we try to set the exposure controls.
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (opModeIsActive() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (opModeIsActive()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }

            exposureControl.setExposure((long) exposureMillis, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}
