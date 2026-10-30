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
    public static final double ORIGIN_DEADBAND_MM = 3.0;
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
        resetEncoders();
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
        fieldDrive(forward, strafe, rotate, botHeading);
    }

    /**
     * Turns "field" commands (relative to the field) into "robot" commands (relative to
     * the way the robot is facing) by rotating them by the robot's heading, then drives.
     * It does NOT read the odometer, so callers that already have the heading avoid a second read.
     */
    private void fieldDrive(double forward, double strafe, double rotate, double botHeading) {
        double rotStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        mecanumDrive(rotForward, rotStrafe, rotate);
    }

    // ---- Tuning values for driveToLocation (change these while testing) ----
    // kP: "how hard to push per unit of error". Bigger = faster approach, but too big overshoots.
    private static final double DRIVE_KP = 0.002;       // motor power per mm of distance left
    private static final double TURN_KP = 1.5;          // motor power per radian of angle left
    // kD: "brake in proportion to speed". Bigger = softer landing, too big = sluggish.
    private static final double DRIVE_KD = 0.0002;      // power removed per mm/sec of speed
    private static final double TURN_KD = 0.1;          // power removed per radian/sec of spin
    // kS: the smallest power that actually makes the robot move (it must beat friction).
    private static final double DRIVE_KS = 0.10;
    private static final double TURN_KS = 0.08;
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MAX_TURN_POWER = 0.4;
    // Most the command may change per loop, so the robot ramps instead of lurching.
    private static final double MAX_POWER_CHANGE_PER_LOOP = 0.1;
    // Must be "close enough" this many loops in a row before we say "arrived".
    private static final int SETTLE_LOOPS = 5;
    // If driveToLocation was not called for this long, start fresh (new move).
    private static final long STALE_NANOS = 200_000_000L;

    private double lastDestX, lastDestY, lastDestHeadDeg;
    private double lastForward, lastStrafe, lastRotate;
    private int settledLoops = 0;
    private long lastCallNanos = 0;
    private boolean hasLastTarget = false;

    /**
     * PD controller with a minimum power:
     *   power = kP * error  -  kD * speed
     *   - kP * error pushes toward the target (far away = push hard, close = push gently).
     *   - kD * speed is a brake: the faster we are moving, the more we ease off,
     *     so we slow down BEFORE reaching the target instead of flying past it.
     *   - If the result is too weak to overcome friction (below kS) while we are still
     *     pushing the right way, we bump it up to kS so the robot doesn't stall short.
     */
    private double pdWithMinPower(double error, double speed, double kP, double kD, double kS, double maxPower) {
        double power = kP * error - kD * speed;
        boolean pushingTowardTarget = power * error > 0;
        if (pushingTowardTarget && Math.abs(power) < kS) {
            power = Math.copySign(kS, power);
        }
        return Range.clip(power, -maxPower, maxPower);
    }

    /** Moves current toward wanted by at most maxStep (a "speed limit" on how fast power changes). */
    private double slew(double current, double wanted, double maxStep) {
        return current + Range.clip(wanted - current, -maxStep, maxStep);
    }

    public boolean driveToLocation(double destX, double destY, double destHeadDeg) {
        update();
        if (odo == null) return true;

        // Start fresh if this is a new target or it has been a while since we were called.
        long now = System.nanoTime();
        boolean newMove = !hasLastTarget
                || destX != lastDestX || destY != lastDestY || destHeadDeg != lastDestHeadDeg
                || now - lastCallNanos > STALE_NANOS;
        if (newMove) {
            settledLoops = 0;
            lastForward = 0;
            lastStrafe = 0;
            lastRotate = 0;
            lastDestX = destX;
            lastDestY = destY;
            lastDestHeadDeg = destHeadDeg;
            hasLastTarget = true;
        }
        lastCallNanos = now;

        Pose2D pos = odo.getPosition();
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        double currentHeadingRad = pos.getHeading(AngleUnit.RADIANS);

        // How fast we are moving right now (used for braking).
        double velX = odo.getVelX(DistanceUnit.MM);          // mm per second
        double velY = odo.getVelY(DistanceUnit.MM);          // mm per second
        double velHeading = odo.getHeadingVelocity();        // radians per second

        double errorX = destX - currentX;
        double errorY = destY - currentY;
        double destHeadRad = Math.toRadians(destHeadDeg);
        // Pythagorean theorem: straight-line distance left to the target.
        double distance = Math.hypot(errorX, errorY);

        // Turn error, wrapped into -180..+180 degrees so we always turn the short way.
        double headingError = destHeadRad - currentHeadingRad;
        while (headingError > Math.PI) {
            headingError -= 2 * Math.PI;
        }
        while (headingError < -Math.PI) {
            headingError += 2 * Math.PI;
        }

        boolean atPosition = distance <= ORIGIN_DEADBAND_MM;
        boolean atHeading = Math.abs(headingError) <= ORIGIN_HEADING_DEADBAND_RAD;

        double wantForward = 0.0;
        double wantStrafe = 0.0;
        double wantRotate = 0.0;

        if (!atPosition) {
            wantForward = pdWithMinPower(errorX, velX, DRIVE_KP, DRIVE_KD, DRIVE_KS, MAX_DRIVE_POWER);
            wantStrafe = pdWithMinPower(errorY, velY, DRIVE_KP, DRIVE_KD, DRIVE_KS, MAX_DRIVE_POWER);
        }
        if (!atHeading) {
            wantRotate = pdWithMinPower(headingError, velHeading, TURN_KP, TURN_KD, TURN_KS, MAX_TURN_POWER);
        }

        // Ramp the commands so power changes smoothly. Stopping (all zeros) is always allowed instantly.
        boolean stopping = atPosition && atHeading;
        lastForward = stopping ? 0 : slew(lastForward, wantForward, MAX_POWER_CHANGE_PER_LOOP);
        lastStrafe = stopping ? 0 : slew(lastStrafe, wantStrafe, MAX_POWER_CHANGE_PER_LOOP);
        lastRotate = stopping ? 0 : slew(lastRotate, wantRotate, MAX_POWER_CHANGE_PER_LOOP);

        fieldDrive(lastForward, -lastStrafe, -lastRotate, currentHeadingRad);

        // Only report "arrived" after being close enough for several loops in a row,
        // so one lucky noisy reading while still moving doesn't count.
        settledLoops = stopping ? settledLoops + 1 : 0;
        return settledLoops >= SETTLE_LOOPS;
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
