package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Vanguard robot specific drive class inheriting common mecanum drive calculations,
 * field-centric control, and odometry navigation from PCMSRoboticsDrive.
 */
public class VanguardDrive extends PCMSRoboticsDrive {

    public VanguardDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    protected void initMotors(HardwareMap hardwareMap) {
        super.initMotors(hardwareMap);
        // Custom motor directions or motor names for the Vanguard robot can be configured here
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    protected void initOdometer(HardwareMap hardwareMap) {
        super.initOdometer(hardwareMap);
        // Custom odometry offsets or pod directions for the Vanguard robot can be configured here
        odo.setOffsets(-162.0, -120.0, DistanceUnit.MM);
    }
}
