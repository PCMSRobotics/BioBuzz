package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleopBioBuzz", group = "StarterBot")
public class TeleopBioBuzz extends OpMode {
    final double FEED_TIME_SECONDS = 0.10;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_TARGET_VELOCITY_LONGSHOT = 1444.1;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double LAUNCHER_MIN_VELOCITY_LONGSHOT = 1400.7;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo flylight = null;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState;

    // Drive wheel and state variables
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    boolean isLongShot = false;

    // --- NEW TELEMETRY VARIABLES ---
    double lastLaunchVelocity = 0.0;
    int launchCount = 0;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        flylight = hardwareMap.get(Servo.class, "flylight");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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

        // --- UPDATED TELEMETRY SECTION ---
        telemetry.addData("State", launchState);
        telemetry.addData("Current Velocity", launcher.getVelocity());
        telemetry.addData("Last Launch Velocity", lastLaunchVelocity);
        telemetry.addData("Total Launches", launchCount);
        telemetry.update();
    }

    @Override
    public void stop() {}

    void mecanumDrive(double forward, double strafe, double rotate){
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
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
                // --- RECORD DATA HERE ---
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