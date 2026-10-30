package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Base Mecanum Drive class for PCMS Robotics robots.
 * Provides field-centric driving, point-to-point navigation (driveToLocation),
 * and GoBilda Pinpoint odometry integration.
 * Robot-specific implementations (such as VanguardDrive) can extend this class
 * and override motor or odometer initialization as needed.
 */
public class PCMSRoboticsDrive {
    public static final double ORIGIN_DEADBAND_MM = 10.0;
    public static final double ORIGIN_HEADING_DEADBAND_RAD = Math.toRadians(3.0);

    protected DcMotor leftFrontDrive;
    protected DcMotor rightFrontDrive;
    protected DcMotor leftBackDrive;
    protected DcMotor rightBackDrive;
    protected GoBildaPinpointDriver odo = null;

    public PCMSRoboticsDrive(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initOdometer(hardwareMap);
    }

    /**
     * Initializes drive motors. Override in subclasses for custom motor names or directions.
     */
    protected void initMotors(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initializes Pinpoint odometer. Override in subclasses for custom offsets or pod settings.
     */
    protected void initOdometer(HardwareMap hardwareMap) {
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odometer");
            odo.setOffsets(0, 0, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.resetPosAndIMU();
        } catch (Exception e) {
            odo = null;
        }
    }

    public void update() {
        if (odo != null) {
            odo.update();
        }
    }

    public void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }

    public void mecanumDriveField(double forward, double strafe, double rotate) {
        update();
        double botHeading = (odo != null) ? odo.getPosition().getHeading(AngleUnit.RADIANS) : 0.0;

        double rotStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        mecanumDrive(rotForward, rotStrafe, rotate);
    }

    public boolean driveToLocation(double destX, double destY, double destHeadDeg) {
        update();
        if (odo == null) return true;

        Pose2D pos = odo.getPosition();
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        double currentHeadingRad = pos.getHeading(AngleUnit.RADIANS);

        double destHeadRad = Math.toRadians(destHeadDeg);
        double distance = Math.hypot(destX - currentX, destY - currentY);

        double driveGain = 0.001;
        double turnGain = 0.5;

        double headingError = destHeadRad - currentHeadingRad;
        while (headingError > Math.PI) {
            headingError -= 2 * Math.PI;
        }
        while (headingError < -Math.PI) {
            headingError += 2 * Math.PI;
        }

        double targetForward = 0.0;
        double targetStrafe = 0.0;
        double targetRotate = 0.0;

        boolean atPosition = distance <= ORIGIN_DEADBAND_MM;
        boolean atHeading = Math.abs(headingError) <= ORIGIN_HEADING_DEADBAND_RAD;

        if (!atPosition) {
            targetForward = Range.clip((destX - currentX) * driveGain, -0.5, 0.5);
            targetStrafe = Range.clip((destY - currentY) * driveGain, -0.5, 0.5);
        }

        if (!atHeading) {
            targetRotate = Range.clip(headingError * turnGain, -0.4, 0.4);
        }

        mecanumDriveField(targetForward, -targetStrafe, -targetRotate);

        return atPosition && atHeading;
    }

    public boolean driveToLocation(Pose2D targetPose) {
        if (targetPose == null) return true;
        return driveToLocation(
                targetPose.getX(DistanceUnit.MM),
                targetPose.getY(DistanceUnit.MM),
                targetPose.getHeading(AngleUnit.DEGREES)
        );
    }

    public boolean driveToOrigin() {
        return driveToLocation(0, 0, 0);
    }

    public void resetEncoders() {
        if (leftBackDrive != null && rightBackDrive != null && leftFrontDrive != null && rightFrontDrive != null) {
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetPosAndIMU() {
        if (odo != null) {
            odo.resetPosAndIMU();
        }
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

    public DcMotor getLeftFrontDrive() {
        return leftFrontDrive;
    }

    public DcMotor getRightFrontDrive() {
        return rightFrontDrive;
    }

    public DcMotor getLeftBackDrive() {
        return leftBackDrive;
    }

    public DcMotor getRightBackDrive() {
        return rightBackDrive;
    }
}
