package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoOpModeClaw extends LinearOpMode {

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
    final double CLAW_OPEN = 0.65;
    final double CLAW_CLOSED = 0.95;

    final double WRIST_FOLDED_IN  = 0.85;
    final double WRIST_FOLDED_OUT = 0.5;

    DcMotor lDrive, rDrive, arm, slide;
    Servo claw;
    IMU imu;

    private DcMotor setupDriveMotor(String label, DcMotorSimple.Direction direction) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setDirection(direction); // both our motors are forward
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
    }

    private Servo setupClawServo(String label) {
        Servo claw = hardwareMap.get(Servo.class, label);
        claw.setPosition(CLAW_OPEN);
        return claw;
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

    private void openClaw() {
        claw.setPosition(CLAW_OPEN);
    }

    private void closeClaw() {
        claw.setPosition(CLAW_CLOSED);
    }

    private void moveArm(int position) {
        arm.setTargetPosition(position);
        arm.setPower(1.0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void extendSlide() {
        slide.setTargetPosition((arm.getCurrentPosition() > ARM_SCORE_SAMPLES) ? 900 : -1700);
        slide.setPower(1.0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void retractSlide() {
        slide.setTargetPosition(0);
        slide.setPower(1.0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void collectMode() {
        moveArm(ARM_COLLECT);
        openClaw();
        sleep(2000);
    }

    private void scoreMode() {
        moveArm(ARM_SCORE_SAMPLES);
        sleep(2000);
    }

    private void homeMode() {
        moveArm(ARM_INITIAL);
        closeClaw();
        sleep(2000);
    }

    private void setupDevices() {
        lDrive = setupDriveMotor("l_drive", DcMotorSimple.Direction.REVERSE);
        rDrive = setupDriveMotor("r_drive", DcMotorSimple.Direction.FORWARD);
        claw = setupClawServo("claw");
        arm = setupDriveMotor("arm", DcMotorSimple.Direction.FORWARD);
        slide = setupDriveMotor("slide", DcMotorSimple.Direction.FORWARD);
        imu = setupIMU("imu");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setupDevices(); // connect devices to their respective global objects

        closeClaw(); // make sure the intake is not running at startup
        // the arm should already be the in the correct position

        waitForStart();
        // START CODE HERE, WE'VE SET UP THE NEEDED DEVICES ABOVE
        // you can ignore the above functions, unless you're interested
        // in understanding how the code works
        // for now it just drives in a square

        // arm:
        // collectMode(), scoreMode(), homeMode()
        //  : will set the robot into the specified mode. waiting until the transformation is
        //  : complete to move onto the next line of code
        // closeClaw(), openClaw()
        //  : open or close the claw
        // extendSlide(), retractSlide()
        //  : extend or retract the slide

        // drive:
        // driveFor(distance in inches, speed)
        //  : move in a straight line
        // turnFor(angle in degrees, speed)
        //  : turn at the specified angle

        moveArm(ARM_INITIAL + 5);

        closeClaw();
        driveFor(-13, 0.5);
        sleep(250);
        turnFor(-135, 0.5);
        scoreMode();
        //The robot has moved forward extended the arm to score and turned 45 degrees

        sleep(250);
        moveArm(140 * (int) ARM_TPD);
        sleep(250);
        extendSlide();
        sleep(1000);
        driveFor(10,0.5);
        sleep(250);
        turnFor(70, 0.5);
        //The robot lines up with the basket

        sleep(250);
        extendSlide();
        //sleep(250);
        sleep(250);
        driveFor(37, 0.5);

        //The robot drives forward to score

        openClaw();
        sleep(1000);
        retractSlide();
        sleep(250);
        driveFor(-15,0.5);
        sleep(250);
        //The robot reverses from its position
        turnFor(-115,0.5);

        //Robot moves to pick up second sample
        sleep(250);
        driveFor(14,0.5);
        sleep(1000);
        moveArm(268 * (int) ARM_TPD);
        sleep(1000);
        closeClaw();
        sleep(500);

        //Robot lines up to score second sample
        scoreMode();
        turnFor(130,0.5);
        sleep(250);
        moveArm(140 * (int) ARM_TPD);
        sleep(250);
        extendSlide();
        sleep(1000);
        driveFor(23,0.5);
        sleep(250);
        openClaw();
        sleep(250);
        retractSlide();
        sleep(1000);
        homeMode();
    }
}