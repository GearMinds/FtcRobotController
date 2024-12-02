package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Random;

@Autonomous(name="Autonomous Mode", group="Robot")
public class AutoOpMode extends LinearOpMode {

    /*
    yellowjacket motor PPR = 1425.1 = CPR / 4
    wheel diameter for both normal and mech wheels = 4.0 inches
    CPI = CPR * inches = counts per inch
     */
    private final double CPI = (1425.2 / 4.0) / (4.0 * 3.14159);

    DcMotor lDrive, rDrive;
    IMU imu;

    private DcMotor setupDriveMotor(String label, DcMotorSimple.Direction direction) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setDirection(direction); // both our motors are forward
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
    }

    public IMU setupIMU(String label) {
        IMU imu = hardwareMap.get(IMU.class, label);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        return imu;
    }

    private void setDriveMotorsTargetPosition(double inches) {
        int lPosition = lDrive.getCurrentPosition() + (int) (inches * CPI);
        int rPosition = rDrive.getCurrentPosition() + (int) (inches * CPI);
        lDrive.setTargetPosition(lPosition);
        rDrive.setTargetPosition(rPosition);
    }

    private void setDriveMotorsMode(DcMotor.RunMode runMode) {
        lDrive.setMode(runMode);
        rDrive.setMode(runMode);
    }

    private void setDriveMotorsPower(double power) {
        lDrive.setPower(power * -1.0);
        rDrive.setPower(power);
    }

    private boolean driveMotorsAreBusy() {
        return lDrive.isBusy() && rDrive.isBusy();
    }

    private void driveFor(double inches, double speed) {
        if (opModeIsActive()) {

            setDriveMotorsTargetPosition(inches);
            setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
            setDriveMotorsPower(Math.abs(speed));

            while (opModeIsActive() && driveMotorsAreBusy());

            setDriveMotorsPower(0);
            setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double getCorrectionPower(double error, double gain, double max) {
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        return Range.clip(Range.clip(error * gain, -1.0, 1.0), -max, max);
    }

    private void turnFor(double angle, double speed) {
        double power, error = angle - getHeading();

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            error = angle - getHeading();
            power = getCorrectionPower(error, 0.02, speed);

            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.update();

            lDrive.setPower(power);
            rDrive.setPower(-power);
        }

        setDriveMotorsPower(0);
        imu.resetYaw();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        lDrive = setupDriveMotor("l_drive", DcMotorSimple.Direction.REVERSE);
        rDrive = setupDriveMotor("r_drive", DcMotorSimple.Direction.FORWARD);
        imu = setupIMU("imu");

        waitForStart();

        while (opModeIsActive()) {
            driveFor(10, 0.5);
            sleep(2000);

            turnFor(90, 0.5);
            sleep(2000);
        }
    }
}
