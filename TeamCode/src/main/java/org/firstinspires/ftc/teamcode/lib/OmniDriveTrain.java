package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OmniDriveTrain {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public OmniDriveTrain() {
        this("fl", "fr", "bl", "br");
    }

    public OmniDriveTrain(String frontLeftLabel, String frontRightLabel, String backLeftLabel, String backRightLabel) {
        this.frontLeft = hardwareMap.get(DcMotor.class, frontLeftLabel);
        this.frontRight = hardwareMap.get(DcMotor.class, frontRightLabel);
        this.backLeft = hardwareMap.get(DcMotor.class, backLeftLabel);
        this.backRight = hardwareMap.get(DcMotor.class, backRightLabel);
    }

    public void setLeftPower(double power) {
        this.frontLeft.setPower(power);
        this.backLeft.setPower(power);
    }

    public void setRightPower(double power) {
        this.frontRight.setPower(power);
        this.backRight.setPower(power);
    }

    public void setStrafeLeftPower(double power) {
        this.frontRight.setPower(power);
        this.backRight.setPower(-power);
        this.frontLeft.setPower(-power);
        this.backLeft.setPower(power);
    }

    public void setStrafeRightPower(double power) {
        this.frontRight.setPower(-power);
        this.backRight.setPower(power);
        this.frontLeft.setPower(power);
        this.backLeft.setPower(-power);
    }
}