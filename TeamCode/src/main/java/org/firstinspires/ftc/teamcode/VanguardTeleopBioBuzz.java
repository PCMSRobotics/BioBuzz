package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name = "TeleopBioBuzz", group = "StarterBot")
public class VanguardTeleopBioBuzz extends OpMode {
    final double FEED_TIME_SECONDS = 0.10;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1126;
    final double LAUNCHER_TARGET_VELOCITY_LONGSHOT = 1444.1;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double LAUNCHER_MIN_VELOCITY_LONGSHOT = 1400.7;
    final double DEADBAND = 0.1;

    final double PARKING_X = 614;
    final double PARKING_Y = 23;
    final double PARKING_HEAD = 0;

    final double SHOOTING_X = 470;
    final double SHOOTING_Y = 1300;
    final double SHOOTING_HEAD = 0;

    // Declare OpMode members.
    private PCMSRoboticsDrive drive;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo flylight = null;

    ElapsedTime feederTimer = new ElapsedTime();
    double oldTime = 0;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState;

    boolean isLongShot = false;

    // --- TELEMETRY VARIABLES ---
    double lastLaunchVelocity = 0.0;
    int launchCount = 0;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        drive = new VanguardDrive(hardwareMap);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        flylight = hardwareMap.get(Servo.class, "flylight");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        float stickLX, stickLY, stickRX;
        drive.update();

        if (gamepad1.a) {
            drive.resetPosAndIMU();
        }

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = (loopTime > 0) ? (1 / loopTime) : 0;
        oldTime = newTime;

        // Implement a deadband area for stick centering
        stickLX = (Math.abs(gamepad1.left_stick_x) < DEADBAND) ? 0 : gamepad1.left_stick_x;
        stickLY = (Math.abs(gamepad1.left_stick_y) < DEADBAND) ? 0 : gamepad1.left_stick_y;
        stickRX = (Math.abs(gamepad1.right_stick_x) < DEADBAND) ? 0 : gamepad1.right_stick_x;

        if (gamepad1.dpad_down) {
            drive.driveToLocation(0, 0, 0);
        } else if (gamepad1.left_trigger_pressed) {
            drive.mecanumDrive(-stickLY, stickLX, stickRX);
        } else if (gamepad1.dpad_up) {
            drive.driveToLocation(SHOOTING_X, SHOOTING_Y, SHOOTING_HEAD);
        } else if (gamepad1.dpad_right) {
            drive.driveToLocation(PARKING_X, PARKING_Y, PARKING_HEAD);
        } else if (gamepad1.dpad_left) {
            drive.driveToLocation(2751, 1306, 180);
        } else {
            drive.mecanumDriveField(-stickLY, stickLX, stickRX);
        }

        if (gamepad1.y) {
            isLongShot = false;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            flylight.setPosition(0.3);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
            flylight.setPosition(0.5);
        } else if (gamepad1.x) {
            isLongShot = true;
            flylight.setPosition(0.3);
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_LONGSHOT);
        }

        launch(gamepad1.rightBumperWasPressed());

        // --- TELEMETRY SECTION ---
        GoBildaPinpointDriver odo = drive.getOdo();
        if (odo != null) {
            Pose2D pos = odo.getPosition();
            String posData = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            String velData = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Position", posData);
            telemetry.addData("Velocity", velData);
            telemetry.addData("Distance to Origin", String.format(Locale.US, "%.1f mm", Math.hypot(pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM))));
            telemetry.addData("Drive to Origin", gamepad1.dpad_down ? "ACTIVE (Dpad Down)" : "Inactive");
            telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
        }
        telemetry.addData("REV Hub Frequency", frequency);
        telemetry.addData("State", launchState);
        telemetry.addData("Current Velocity", launcher.getVelocity());
        telemetry.addData("Last Launch Velocity", lastLaunchVelocity);
        telemetry.addData("Total Launches", launchCount);
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    void spin() {
        drive.mecanumDriveField(0, 0, .1);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (!isLongShot) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_LONGSHOT);
                    if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY_LONGSHOT) {
                        launchState = LaunchState.LAUNCH;
                    }
                }
                break;
            case LAUNCH:
                lastLaunchVelocity = launcher.getVelocity();
                launchCount++;

                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
