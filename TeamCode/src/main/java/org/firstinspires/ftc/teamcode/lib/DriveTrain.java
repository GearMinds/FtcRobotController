package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain {

    private DcMotor[] motors = new DcMotor[2];
    // index 0: left motor
    // index 1: right motor

    public DriveTrain() {
        this("left_drive_motor", "right_drive_motor");
    }

    public DriveTrain(String leftMotorLabel, String rightMotorLabel) {
        this.motors[0] = hardwareMap.get(DcMotor.class, leftMotorLabel);
        this.motors[1] = hardwareMap.get(DcMotor.class, rightMotorLabel);

        for (DcMotor motor : this.motors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public DcMotor getLeftMotor() {
        return this.motors[0];
    }

    public DcMotor getRightMotor() {
        return this.motors[1];
    }

    public void setLeftDirection(DcMotorSimple.Direction direction) {
        this.getLeftMotor().setDirection(direction);
    }

    public void setRightDirection(DcMotorSimple.Direction direction) {
        this.getRightMotor().setDirection(direction);
    }

    public void setLeftPower(double power) {
        this.getLeftMotor().setPower(power);
    }

    public void setRightPower(double power) {
        this.getRightMotor().setPower(power);
    }
}
