package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoBioBuzz", group = "Autonomous")
public class AutoBioBuzz extends LinearOpMode {

    // Hardware
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private DistanceSensor distance;
    private IMU imu;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // Constants
    private static final double COUNTS_PER_10_CM = 168.1889764;
    private static final int DESIRED_DISTANCE_INCHES = 30;
    private static final int DESIRED_TAG_ID_BLUE = 20;
    private static final int DESIRED_TAG_ID_RED = 24;

    // Control Gains & Limits
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.02;
    private static final double TURN_GAIN = 0.01;

    private static final double MAX_AUTO_SPEED = 0.2;
    private static final double MAX_AUTO_STRAFE = 0.2;
    private static final double MAX_AUTO_TURN = 0.2;

    // State & Drive Variables
    private enum State {
        IDLE,
        DRIVE_TO_TARGET,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_HOME,
        TURN_LEFT,
        TURN_RIGHT,
        PREP,
        SHOOT,
        WAIT,
        STOP
    }

    private State currentState = State.IDLE;
    private double driveSpeed = 0.2;
    private int cameraGain = 125;

    private double xPower = 0;
    private double yPower = 0;
    private double rxPower = 0;

    private int targetFoundId = 0;
    private double rangeError = 100000;
    private double headingError = 0;
    private double yawError = 0;

    private YawPitchRollAngles orientation;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int shootStep = 0;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            processAprilTags();
            updateStateLogic();
            applyDrivePowers();
            updateTelemetryData();
        }

        // Clean up vision portal on stop
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initHardware() {
        // Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotor.class, "launcher");

        // Servos
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Sensors
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        imu = hardwareMap.get(IMU.class, "imu");

        // Motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behaviors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        // IMU Configuration
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();
        orientation = imu.getRobotYawPitchRollAngles();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, cameraGain);
    }

    private void processAprilTags() {
        targetFoundId = 0;
        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Check if this detection is a single tag (not a cluster)
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                // Check if the tag is recognized in the tag library
                if (singleDet.metadata != null) {
                    if (singleDet.id == DESIRED_TAG_ID_BLUE || singleDet.id == DESIRED_TAG_ID_RED) {
                        targetFoundId = singleDet.id;
                        desiredTag = detection; // ftcPose is accessible on base AprilTagDetection
                        break;
                    }
                }
            }
        }

        if (desiredTag != null && desiredTag.ftcPose != null) {
            rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
            headingError = desiredTag.ftcPose.bearing;
            yawError = desiredTag.ftcPose.yaw;
        }
    }
    private void updateStateLogic() {
        switch (currentState) {
            case IDLE:
                xPower = 0; yPower = 0; rxPower = 0;
                if (targetFoundId != 0) {
                    transitionTo(State.DRIVE_TO_TARGET);
                }
                break;

            case DRIVE_TO_TARGET:
                if (targetFoundId != 0) {
                    rxPower = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    xPower = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    yPower = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    if (rangeError <= DESIRED_DISTANCE_INCHES && Math.abs(headingError) < 5 && Math.abs(yawError) < 5) {
                        resetEncoders();
                        transitionTo(State.DRIVE_TO_TARGET_2);
                    }
                } else {
                    xPower = 0; yPower = 0; rxPower = 0;
                    resetEncoders();
                    transitionTo(State.DRIVE_TO_TARGET_2);
                }
                break;

            case DRIVE_TO_TARGET_2:
                xPower = 0; yPower = driveSpeed; rxPower = 0;
                if (rightFrontDrive.getCurrentPosition() >= 15 * COUNTS_PER_10_CM) {
                    transitionTo(State.PREP);
                }
                break;

            case PREP:
                xPower = 0; yPower = 0; rxPower = 0;
                launcher.setPower(0.5);
                if (stateTimer.seconds() >= 2.0) {
                    shootStep = 0;
                    transitionTo(State.SHOOT);
                }
                break;

            case SHOOT:
                // Non-blocking sequential feeder pulses
                runShootingSequence();
                break;

            case TURN_RIGHT:
                xPower = 0; yPower = 0; rxPower = driveSpeed;
                if (orientation.getYaw(AngleUnit.DEGREES) <= -35) {
                    resetEncoders();
                    transitionTo(State.STOP);
                }
                break;

            case TURN_LEFT:
                xPower = 0; yPower = 0; rxPower = -driveSpeed;
                if (orientation.getYaw(AngleUnit.DEGREES) >= 40) {
                    resetEncoders();
                    transitionTo(State.STOP);
                }
                break;

            case DRIVE_HOME:
                xPower = 0; yPower = -driveSpeed; rxPower = 0;
                stopFeedersAndLauncher();
                break;

            case DRIVE_TO_TARGET_3:
                driveSpeed = 0.4;
                xPower = 0; yPower = -driveSpeed; rxPower = 0;
                if (rightFrontDrive.getCurrentPosition() <= -40.67 * COUNTS_PER_10_CM) {
                    transitionTo(State.STOP);
                }
                break;

            case WAIT:
                xPower = 0; yPower = 0; rxPower = 0;
                if (stateTimer.seconds() >= 5.0) {
                    transitionTo(State.IDLE);
                }
                break;

            case STOP:
            default:
                xPower = 0; yPower = 0; rxPower = 0;
                stopFeedersAndLauncher();
                break;
        }
    }

    private void runShootingSequence() {
        // Non-blocking timer-driven shooting routine replaces nested sleep() calls
        double t = stateTimer.seconds();
        if (shootStep == 0) {
            leftFeeder.setPower(-1); rightFeeder.setPower(1);
            if (t > 0.3) { setFeeders(0, 0); shootStep = 1; stateTimer.reset(); }
        } else if (shootStep == 1) {
            if (t > 1.0) { setFeeders(-1, 1); shootStep = 2; stateTimer.reset(); }
        } else if (shootStep == 2) {
            if (t > 0.1) { setFeeders(0, 0); shootStep = 3; stateTimer.reset(); }
        } else if (shootStep == 3) {
            if (t > 1.0) { setFeeders(-1, 1); shootStep = 4; stateTimer.reset(); }
        } else if (shootStep == 4) {
            if (t > 0.1) { setFeeders(0, 0); shootStep = 5; stateTimer.reset(); }
        } else if (shootStep == 5) {
            if (t > 1.0) {
                resetEncoders();
                transitionTo(State.IDLE);
            }
        }
    }

    private void setFeeders(double left, double right) {
        leftFeeder.setPower(left);
        rightFeeder.setPower(right);
    }

    private void stopFeedersAndLauncher() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        launcher.setPower(0);
    }

    private void transitionTo(State newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void applyDrivePowers() {
        leftFrontDrive.setPower(yPower + xPower + rxPower);
        leftBackDrive.setPower((yPower - xPower) + rxPower);
        rightFrontDrive.setPower((yPower - xPower) - rxPower);
        rightBackDrive.setPower((yPower + xPower) - rxPower);

        orientation = imu.getRobotYawPitchRollAngles();
    }

    private void resetEncoders() {
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setManualExposure(int exposureMs, int gain) {
        if (visionPortal == null) return;

        if (!visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
            while (!isStopRequested() && !visionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void updateTelemetryData() {
        telemetry.addData("State", currentState);
        telemetry.addData("Drive Powers (X, Y, RX)", "%.2f, %.2f, %.2f", xPower, yPower, rxPower);
        telemetry.addData("Encoders (LB : RB)", "%d : %d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.addData("Heading (Yaw)", "%.2f Deg", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Target Found ID", targetFoundId);
        telemetry.addData("Range Error", "%.2f in", rangeError);
        telemetry.addData("Heading Error", "%.2f Deg", headingError);
        telemetry.addData("Yaw Error", "%.2f Deg", yawError);
        telemetry.addData("Distance Sensor", "%.2f cm", distance.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}