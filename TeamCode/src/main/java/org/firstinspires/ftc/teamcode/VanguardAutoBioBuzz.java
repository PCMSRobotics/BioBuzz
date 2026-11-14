package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Autonomous OpMode for BioBuzz Season.
 * Demonstrates a simple state machine for autonomous driving and launcher operation.
 */
@Autonomous(name = "VanguardAutoBioBuzz", group = "Autonomous")
public class VanguardAutoBioBuzz extends LinearOpMode {

    // --- Target Locations Map ---
    final double SHOOTING_X = 470;
    final double SHOOTING_Y = 1300;
    final double SHOOTING_HEAD = 0;

    final double PARKING_X = 614;
    final double PARKING_Y = 23;
    final double PARKING_HEAD = 0;

    // Map variable defined in Constants area for autonomous target positions
    final Map<String, Pose2D> TARGET_MAP = new HashMap<String, Pose2D>() {{
        put("SHOOTING1", new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
        put("SHOOTING2", new Pose2D(DistanceUnit.MM, -2980, 233, AngleUnit.DEGREES, -179));
        put("PRE_PICKUP", new Pose2D(DistanceUnit.MM, -36, -18, AngleUnit.DEGREES, -89));
        put("PRE_PARKING", new Pose2D(DistanceUnit.MM, -63, -20, AngleUnit.DEGREES, -89));
        put("PICKUP", new Pose2D(DistanceUnit.MM, 0, -1306, AngleUnit.DEGREES, -89));
        put("PARKING", new Pose2D(DistanceUnit.MM, -2037, -1280, AngleUnit.DEGREES, -93));
    }};

    // Motor & Servo Speed Constants
    final double LAUNCHER_TARGET_VELOCITY = 1250;
    final double LAUNCHER_MIN_VELOCITY = 1200;
    final double FEED_TIME_SECONDS = 0.10;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    double intakePower;

    // Hardware Members
    private PCMSRoboticsDrive drive;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo rightintakeservo = null;
    private CRServo leftintakeservo = null;
    private CRServo windmillservo = null;

    // State Machine Variables
    private enum State {
        IDLE,
        DRIVE_TO_SHOOTING1,
        DRIVE_TO_SHOOTING2,
        DRIVE_TO_SHOOTING1_2,
        DRIVE_TO_SHOOTING2_2,
        DRIVE_TO_PRE_PRE_PICKUP,
        DRIVE_TO_PRE_PICKUP,
        DRIVE_TO_PICKUP,
        DRIVE_TO_PRE_PARKING,
        DRIVE_TO_PARKING,
        SHOOT1,
        SHOOT2,
        SHOOT3,
        STOP
    }

    private State currentState = State.SHOOT1;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int shootStep = 0;

    // Telemetry Tracking Variables (Matching TeleOp)
    private double oldTime = 0.0;
    private double lastLaunchVelocity = 0.0;
    private int launchCount = 0;


    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();

        stateTimer.reset();
        transitionTo(State.DRIVE_TO_SHOOTING1);

        while (opModeIsActive()) {
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = (loopTime > 0) ? (1 / loopTime) : 0;
            oldTime = newTime;

            drive.update();
            updateStateLogic();
            updateTelemetryData(frequency);
        }

        stopFeedersAndLauncher();
    }

    private void initHardware() {
        drive = new VanguardDrive(hardwareMap);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftintakeservo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightintakeservo = hardwareMap.get(CRServo.class, "right_intake_servo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmillservo = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftintakeservo.setPower(STOP_SPEED);
        rightintakeservo.setPower(STOP_SPEED);
        leftintakeservo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void updateStateLogic() {
        switch (currentState) {
            case IDLE:
                stopFeedersAndLauncher();
                break;

            case DRIVE_TO_SHOOTING1:
                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("SHOOTING1"))) {
                    transitionTo(State.SHOOT1);
                }
                break;
            case DRIVE_TO_SHOOTING2:
                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("SHOOTING2"))) {
                    transitionTo(State.SHOOT2);
                }
                break;

            case DRIVE_TO_SHOOTING1_2:
                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("SHOOTING1"))) {
                    transitionTo(State.DRIVE_TO_SHOOTING2_2);
                }
                break;

            case DRIVE_TO_SHOOTING2_2:
                // Use driveToLocation with the target location map from Constants area
                launcher.setVelocity(0);
                if (drive.driveToLocation(TARGET_MAP.get("SHOOTING2"))) {
                    transitionTo(State.SHOOT3);
                }
                break;

            case DRIVE_TO_PRE_PRE_PICKUP:
                launcher.setVelocity(0);
                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("SHOOTING1"))) {
                    transitionTo(State.DRIVE_TO_PRE_PICKUP);
                }
                break;

            case DRIVE_TO_PRE_PICKUP:
                // Use driveToLocation with the target location map from Constants area
                launcher.setVelocity(0);
                if (drive.driveToLocation(TARGET_MAP.get("PRE_PARKING"))) {
                    intake.setPower(intakePower);
                    rightintakeservo.setPower(intakePower);
                    leftintakeservo.setPower(intakePower);
                    transitionTo(State.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PRE_PARKING:
                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("PRE_PICKUP"))) {
                    transitionTo(State.DRIVE_TO_PARKING);
                }
                break;
            case DRIVE_TO_PICKUP:

                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("PICKUP"))) {
                    intake.setPower(intakePower);
                    rightintakeservo.setPower(intakePower);
                    leftintakeservo.setPower(intakePower);
                    transitionTo(State.DRIVE_TO_PRE_PARKING);
                }
                break;
            case DRIVE_TO_PARKING:
                // Use driveToLocation with the target location map from Constants area
                if (drive.driveToLocation(TARGET_MAP.get("PARKING"))) {
                    transitionTo(State.STOP);
                }
                break;
            //           case PREP:
//                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            //             if (stateTimer.seconds() >= 2.0) {
            //                  shootStep = 0;
            //                   transitionTo(State.SHOOT1);
//                }
            //               break;

            case SHOOT1:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    runShootingSequence();
                    launchCount++;
                }
                if (launchCount == 3) {
                    launchCount = 0;
                    transitionTo(State.DRIVE_TO_PRE_PICKUP);
                }
                break;

            case SHOOT2:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    runShootingSequence();

                }
                transitionTo(State.DRIVE_TO_PRE_PRE_PICKUP);

                break;

            case SHOOT3:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    runShootingSequence();

                }
                transitionTo(State.DRIVE_TO_PARKING);

                break;


            case STOP:
            default:
                stopFeedersAndLauncher();
                drive.mecanumDrive(0, 0, 0);
                break;
        }
    }

    private void runShootingSequence() {
        if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillservo.setPower(-1);
            intake.setPower(.75);
            while (launcher.getVelocity() >= LAUNCHER_TARGET_VELOCITY) {
                sleep(1);
            }
        } else {
            windmillservo.setPower(0);
        }
    }

    private void stopFeeders() {
        leftintakeservo.setPower(STOP_SPEED);
        rightintakeservo.setPower(STOP_SPEED);
    }

    private void stopFeedersAndLauncher() {
        stopFeeders();
        if (launcher != null) {
            launcher.setVelocity(STOP_SPEED);
        }
    }

    private void transitionTo(State newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void updateTelemetryData(double frequency) {
        GoBildaPinpointDriver odo = drive.getOdo();
        if (odo != null) {
            Pose2D pos = odo.getPosition();
            String posData = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            String velData = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Position", posData);
            telemetry.addData("Velocity", velData);
            telemetry.addData("Distance to Origin", String.format(Locale.US, "%.1f mm", Math.hypot(pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM))));
            telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
        }
        telemetry.addData("REV Hub Frequency", frequency);
        telemetry.addData("State", currentState);
        telemetry.addData("Current Velocity", (launcher != null) ? launcher.getVelocity() : 0.0);
        telemetry.addData("Last Launch Velocity", lastLaunchVelocity);
        telemetry.addData("Total Launches", launchCount);
        telemetry.update();
    }
}
