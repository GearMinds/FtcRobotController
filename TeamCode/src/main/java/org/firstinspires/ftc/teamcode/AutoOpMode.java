package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Example Auto Mode", group="nice")
public class AutoOpMode extends LinearOpMode {

    /*
    yellowjacket motor PPR = 1425.1 = CPR / 4
    wheel diameter for both normal and mech wheels = 4.0 inches
    CPI = CPR * inches = counts per inch
     */
    private final double CPI = 1425.1 * 4.0 * 4.0;
    private final double DRIVE_SPEED = 0.5;
    private final double TURN_SPEED = 0.5;

    private DcMotor setupDriveMotor(String label) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setDirection(DcMotorSimple.Direction.FORWARD); // both our motors are forward
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return driveMotor;
    }

    private void driveFor(DcMotor[] motors, double speed, double inches) {
        if (opModeIsActive()) {

            int target0 = motors[0].getCurrentPosition(); // TODO: finish
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motors[] = { setupDriveMotor("l_drive"), setupDriveMotor("r_drive") };

        waitForStart();

        while (opModeIsActive()) {
            driveFor(motors, 1.0, 10.0);
        }
    }
}
