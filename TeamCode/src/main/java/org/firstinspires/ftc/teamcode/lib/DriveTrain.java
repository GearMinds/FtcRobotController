package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public double maxPower = 1.0;
    private double speed = 1.0;
    private double gain = 0.01;

    private final Robot robot;

    private final double wheelRadius = 2.5;
    private final double PPR = 537.7;
    private final double CPI = PPR / (2.0 * 3.14159 * wheelRadius);
    private final IMU imu;

    private DcMotor setupDriveMotor(HardwareMap hardwareMap, DcMotorSimple.Direction direction, String label) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setDirection(direction);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
    }

    private IMU setupIMU(HardwareMap hardwareMap, String label) {
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
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int) (inches * CPI));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int) (inches * CPI));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int) (inches * CPI));
        backRight.setTargetPosition(backRight.getCurrentPosition() + (int) (inches * CPI));
    }

    private void setDriveMotorsMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    private void setDriveMotorsPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private boolean driveMotorsAreBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double getCorrectionPower(double error) {
        while (error > 360) error -= 180;
        while (error < -360) error += 180;

        return Range.clip(Range.clip(error * gain, -1.0, 1.0), -speed, speed);
    }

    public DriveTrain(Robot robot) {
        this.frontLeft = setupDriveMotor(robot.hardwareMap, DcMotorSimple.Direction.REVERSE, "fl");
        this.frontRight = setupDriveMotor(robot.hardwareMap, DcMotorSimple.Direction.FORWARD, "fr");
        this.backLeft = setupDriveMotor(robot.hardwareMap, DcMotorSimple.Direction.FORWARD, "bl");
        this.backRight = setupDriveMotor(robot.hardwareMap, DcMotorSimple.Direction.REVERSE, "br");
        this.imu = setupIMU(robot.hardwareMap, "imu");
        this.robot = robot;
    }

    public void setPower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        this.frontLeft.setPower(leftFrontPower * maxPower);
        this.backLeft.setPower(leftBackPower * maxPower);
        this.frontRight.setPower(rightFrontPower * maxPower);
        this.backRight.setPower(rightBackPower * maxPower);
    }

    public void setSpeed(double speed) {
        if (speed > 1.0) {
            speed = 1.0;
        } else if (speed < 0.0) {
            speed = 0.0;
        }
        this.speed = speed;
    }

    public void forward() {
        setPower(speed, speed, speed, speed);
    }

    public void reverse() {
        setPower(-speed, -speed, -speed, -speed);
    }

    public void rotateLeft() {
        setPower(-speed, -speed, speed, speed);
    }

    public void rotateRight() {
        setPower(speed, speed, -speed, -speed);
    }

    public void strafeLeft() {
        setPower(-speed, speed, speed, -speed);
    }

    public void strafeRight() {
        setPower(speed, -speed, -speed, speed);
    }

    public void stop() {
        setPower(0.0, 0.0, 0.0, 0.0);
    }

    public void forwardFor(double inches) {
        if (robot.opModeIsActive()) {

            setDriveMotorsTargetPosition(inches);
            setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
            setDriveMotorsPower(Math.abs(speed));

            while (robot.opModeIsActive() && driveMotorsAreBusy());

            setDriveMotorsPower(0);
            setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void reverseFor(double inches) {
        forwardFor(-inches);
    }

    public void strafeRightFor(double inches) {
        if (robot.opModeIsActive()) {

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int) (inches * CPI));
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + (int) (-inches * CPI));
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + (int) (-inches * CPI));
            backRight.setTargetPosition(backRight.getCurrentPosition() + (int) (inches * CPI));

            setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
            setDriveMotorsPower(Math.abs(speed));

            while (robot.opModeIsActive() && driveMotorsAreBusy());

            setDriveMotorsPower(0);
            setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafeLeftFor(double inches) {
        strafeRightFor(-inches);
    }

    public void rotateRightFor(double angle) {
        double power, error = angle + getHeading();

        while (robot.opModeIsActive() && Math.abs(error) > 1.0) {
            error = angle + getHeading();
            power = getCorrectionPower(error);

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
        }

        setDriveMotorsPower(0);
        imu.resetYaw();
    }

    public void rotateLeftFor(double angle) {
        rotateRightFor(-angle);
    }
}