package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@Config
@TeleOp(name = "VanarchyTeleopBioBuzz", group = "StarterBot")
public class VanarchyTeleopBioBuzz extends OpMode {

    // Launcher constants from reference starterbot - exposed to dashboard / panels via @Config
    public static double LAUNCHER_TARGET_VELOCITY = 1300;
    public static double LAUNCHER_MIN_VELOCITY = 1200;
    public static double CONTROLLER_DEADBAND = 0.1;

    // Declare OpMode members.
    private PCMSRoboticsDrive drive;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    double intakePower;
    double oldTime = 0;

    @Override
    public void init() {
        drive = new VanarchyDrive(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        boolean arrived = false;

        // Implement a deadband area for stick centering
        stickLX = (Math.abs(gamepad1.left_stick_x) < CONTROLLER_DEADBAND) ? 0 : gamepad1.left_stick_x;
        stickLY = (Math.abs(gamepad1.left_stick_y) < CONTROLLER_DEADBAND) ? 0 : gamepad1.left_stick_y;
        stickRX = (Math.abs(gamepad1.right_stick_x) < CONTROLLER_DEADBAND) ? 0 : gamepad1.right_stick_x;

        // Drive controls
        if (gamepad1.dpad_up) {
            arrived = drive.driveToLocation(0, 0, 0);
        } else if (gamepad1.left_bumper) {
            drive.mecanumDriveField(-stickLY, stickLX, stickRX);
        } else {
            drive.mecanumDrive(-stickLY, stickLX, stickRX);
        }

        // Position reset
        if (gamepad1.a) {
            drive.resetPosAndIMU();
        }

        // Intake and launch controls
        // Intake power calculation from triggers
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        // Apply intake power to intake motor and servos
        intake.setPower(intakePower);
        leftIntakeServo.setPower(-intakePower);
        rightIntakeServo.setPower(-intakePower);

        launch();
        drive.update();
        // --- TELEMETRY SECTION ---
        GoBildaPinpointDriver odo = drive.getOdo();
        if (odo != null) {
            Pose2D pos = odo.getPosition();
            String posData = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            String velData = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Position", posData);
            telemetry.addData("Velocity", velData);
            telemetry.addData("Distance to Origin", String.format(Locale.US, "%.1f mm", Math.hypot(pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM))));
            telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
            telemetry.addData("Arrived:", arrived);
        }
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    void launch() {
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
        } else if (gamepad1.dpad_down) {
            windmillServo.setPower(-1);
        } else {
            windmillServo.setPower(0);
        }
    }
}
