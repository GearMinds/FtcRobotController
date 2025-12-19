package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OmniDriveTrain {

    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public double maxPower = 1.0;

    private final double CPI = (1425.2 / 4.0) / (4.0 * 3.14159);

    private DcMotor setupDriveMotor(HardwareMap hardwareMap, String label) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
    }

    public OmniDriveTrain(HardwareMap hardwareMap) {
        this(hardwareMap, "fl", "fr", "bl", "br");
    }

    public OmniDriveTrain(HardwareMap hardwareMap, String frontLeftLabel, String frontRightLabel, String backLeftLabel, String backRightLabel) {
        this.frontLeft = setupDriveMotor(hardwareMap, frontLeftLabel);
        this.frontRight = setupDriveMotor(hardwareMap, frontRightLabel);
        this.backLeft = setupDriveMotor(hardwareMap, backLeftLabel);
        this.backRight = setupDriveMotor(hardwareMap, backRightLabel);
    }

    public void setPower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        this.frontLeft.setPower(leftFrontPower * maxPower);
        this.backLeft.setPower(leftBackPower * maxPower);
        this.frontRight.setPower(rightFrontPower * maxPower);
        this.backRight.setPower(rightBackPower * maxPower);
    }

    public void moveForward() {
        setPower(1.0, 1.0, 1.0, 1.0);
    }

    public void moveBackward() {
        setPower(-1.0, -1.0, -1.0, -1.0);
    }

    public void stop() {
        setPower(0.0, 0.0, 0.0, 0.0);
    }
}