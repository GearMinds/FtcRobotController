package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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
    final double ARM_TPD = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final int ARM_INITIAL       = 0;
    final int ARM_COLLECT       = (int) (250 * ARM_TPD);
    final int ARM_CLEAR_BARRIER = (int) (230 * ARM_TPD);
    final int ARM_SCORE_SAMPLES = (int) (155 * ARM_TPD);
    final int ARM_STRAIGHT_UP   = (int) (120 * ARM_TPD);
    final int ARM_WINCH_ROBOT   = (int) (15 * ARM_TPD);

    // wrist pre-defined constants
    final double WRIST_FOLDED_IN  = 0.85;
    final double WRIST_FOLDED_OUT = 0.5;

    DcMotor lDrive, rDrive, arm;
    CRServo intake;
    Servo wrist;
    IMU imu;

    private DcMotor setupDriveMotor(String label, DcMotorSimple.Direction direction) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setDirection(direction); // both our motors are forward
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
    }

    private CRServo setupIntakeServo(String label) {
        CRServo intake = hardwareMap.get(CRServo.class, label);
        intake.setPower(0); // make sure the intake is not powered on
        return intake;
    }

    private Servo setupWristServo(String label) {
        Servo wrist = hardwareMap.get(Servo.class, label);
        wrist.resetDeviceConfigurationForOpMode();
        wrist.setPosition(WRIST_FOLDED_IN);
        return wrist;
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

    private void intakePull() {
        intake.setPower(-1.0);
    }

    private void intakePush() {
        intake.setPower(1.0);
    }

    private void intakeOff() {
        intake.setPower(0.0);
    }

    private void wristOut() {
        wrist.setPosition(WRIST_FOLDED_OUT);
    }

    private void wristIn() {
        wrist.setPosition(WRIST_FOLDED_IN);
    }

    private void moveArm(int position) {
        arm.setTargetPosition(position);
        arm.setPower(1.0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void collectMode() {
        moveArm(ARM_COLLECT);
        intakePull();
        wristOut();
    }

    private void scoreMode() {
        moveArm(ARM_SCORE_SAMPLES);
    }

    private void homeMode() {
        moveArm(ARM_INITIAL);
        intakeOff();
        wristIn();
    }

    private void setupDevices() {
        lDrive = setupDriveMotor("l_drive", DcMotorSimple.Direction.REVERSE);
        rDrive = setupDriveMotor("r_drive", DcMotorSimple.Direction.FORWARD);
        intake = setupIntakeServo("intake");
        wrist = setupWristServo("wrist");
        arm = setupDriveMotor("arm", DcMotorSimple.Direction.FORWARD);
        imu = setupIMU("imu");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setupDevices(); // connect devices to their respective global objects

        intakeOff(); // make sure the intake is not running at startup
        wristIn(); // make sure the wrist is folded in
        // the arm should already be the in the correct position

        waitForStart();
        // START CODE HERE, WE'VE SET UP THE NEEDED DEVICES ABOVE
        // you can ignore the above functions, unless you're interested
        // in understanding how the code works
        // for now it just drives in a square

        // EXAMPLE 1:
        // start collecting, move forward, turn and score, then fold the robot up
        collectMode();                  // 1. start collecting
        driveFor(10, 1.0); // 2. move forward (10 inches)
        scoreMode();                    // 3. move arm into score position
        turnFor(-45, 0.5);  // 4. turn, perhaps the goal is at a diagonal to the left
        driveFor(3, 0.2);  // 4a. drive slowly the goal may be slightly out of reach
        intakePush();                   // 5. score by pushing out whatever is in the intake feeder
        homeMode();                     // 6. fold the robot back up

        // EXAMPLE 2:
        // run in a square forever 10x10 inches
        // - sleep for 2 seconds between each instruction
        // - run at half speed
        while (opModeIsActive()) { // forever
            driveFor(10, 0.5); // straight line for 10 inches
            sleep(2000);

            turnFor(90, 0.5); // 90 deg turn
            sleep(2000);
        }
    }
}
