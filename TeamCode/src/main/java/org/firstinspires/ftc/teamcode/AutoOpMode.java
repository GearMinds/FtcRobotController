package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Autonomous Mode", group="Robot")
public class AutoOpMode extends LinearOpMode {

    /*
    yellowjacket motor PPR = 1425.1 = CPR / 4
    wheel diameter for both normal and mech wheels = 4.0 inches
    CPI = CPR * inches = counts per inch
     */
    private final double CPI = (1425.2 / 4.0) / (4.0 * 3.14159);

    DcMotor lDrive, rDrive;

    private DcMotor setupDriveMotor(String label, DcMotorSimple.Direction direction) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setDirection(direction); // both our motors are forward
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
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

            while (opModeIsActive() && driveMotorsAreBusy()) {
                telemetry.addData("target", inches * CPI);
                telemetry.addData("l pos", lDrive.getCurrentPosition());
                telemetry.addData("r pos", rDrive.getCurrentPosition());
                telemetry.update();
            }

            setDriveMotorsPower(0);
            setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        lDrive = setupDriveMotor("l_drive", DcMotorSimple.Direction.REVERSE);
        rDrive = setupDriveMotor("r_drive", DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            driveFor(5.0, 0.1);
            sleep(2000);
        }
    }
}
