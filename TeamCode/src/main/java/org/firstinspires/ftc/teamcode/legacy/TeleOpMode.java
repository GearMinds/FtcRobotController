package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpMode extends LinearOpMode {
    // Arm ticks per degree of rotation
    // comes out to around 19.792489314064724
    final double ARM_TPD = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    // constants to use for pre-defined arm positions
    // values taken from sample code
    final int ARM_INITIAL       = 0;
    final int ARM_COLLECT       = (int) (250 * ARM_TPD);
    final int ARM_CLEAR_BARRIER = (int) (230 * ARM_TPD);
    final int ARM_SCORE_SAMPLES = (int) (155 * ARM_TPD);
    final int ARM_STRAIGHT_UP   = (int) (120 * ARM_TPD);
    final int ARM_WINCH_ROBOT   = (int) (15 * ARM_TPD);

    // wrist pre-defined constants
    final double WRIST_FOLDED_IN  = 0.85;
    final double WRIST_FOLDED_OUT = 0.5;

    private DcMotor setupDriveMotor(String label, DcMotorSimple.Direction direction) {
        DcMotor driveMotor = hardwareMap.get(DcMotor.class, label);

        driveMotor.setDirection(direction);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return driveMotor;
    }

    private DcMotor setupArmMotor(String label) {
        DcMotor arm = hardwareMap.get(DcMotor.class, label);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return arm;
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

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lDrive = setupDriveMotor("l_drive", DcMotorSimple.Direction.REVERSE);
        DcMotor rDrive = setupDriveMotor("r_drive", DcMotorSimple.Direction.FORWARD);
        DcMotor arm = setupArmMotor("arm");

        CRServo intake = setupIntakeServo("intake");
        Servo wrist = setupWristServo("wrist");

        int armPosition = ARM_INITIAL;

        telemetry.addLine("Waiting for robot to start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // set up control sticks
            double l = -gamepad1.left_stick_y - gamepad1.right_stick_x;
            double r = -gamepad1.left_stick_y + gamepad1.right_stick_x;
            double overflow = Math.max(Math.abs(l), Math.abs(r));

            if (overflow > 1.0) {
                l /= overflow;
                r /= overflow;
            }

            lDrive.setPower(l / 1.5);
            rDrive.setPower(r / 1.5);

            // control wrist
            if (gamepad1.right_trigger > 0.5) // RT -> fold wrist in
                wrist.setPosition(WRIST_FOLDED_IN);
            if (gamepad1.left_trigger > 0.5) // LT -> fold wrist out
                wrist.setPosition(WRIST_FOLDED_OUT);

            // intake mappings
            if (gamepad1.a) // A -> collect
                intake.setPower(-1.0);
            if (gamepad1.b) // B -> deposit
                intake.setPower(0.5);
            if (gamepad1.x) // X -> off
                intake.setPower(0.0);

            // arm position mappings
            if (gamepad1.y) // Y -> Move arm to score in the basket
                armPosition = ARM_SCORE_SAMPLES;
            if (gamepad1.left_bumper) // LB -> Move arm to clear barrier
                armPosition = ARM_CLEAR_BARRIER;
            if (gamepad1.dpad_up) // D-Up -> prepare arm for hanging
                armPosition = ARM_STRAIGHT_UP;
            if (gamepad1.dpad_down) // D-Down -> winch robot upwards
                armPosition = ARM_WINCH_ROBOT;

            if (gamepad1.right_bumper) { // RB -> Move arm to intake position and turn on intake
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(-1.0);
                armPosition = ARM_COLLECT;
            }

            if (gamepad1.dpad_left) { // D-Left -> Reset arm to original position
                wrist.setPosition(WRIST_FOLDED_IN);
                intake.setPower(0.0);
                armPosition = ARM_INITIAL;
            }

            if (gamepad1.dpad_right) { // D-Right -> Set arm to specimen score configuration
                wrist.setPosition(0.0);
                armPosition = ARM_SCORE_SAMPLES;
            }

            // set target position
            arm.setTargetPosition(armPosition);
            arm.setPower(1.0); // go at full power!
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // send relevant data to driver station
            telemetry.addLine("L Analog: Forward / Backward");
            telemetry.addLine("R Analog: Left / Right");
            telemetry.addLine();
            telemetry.addLine("R Bumper: Feed Mode");
            telemetry.addLine("L Bumper: Clear Lower Barrier");
            telemetry.addLine("Y: Score Mode");
            telemetry.addLine();
            telemetry.addLine("A: Intake Pull");
            telemetry.addLine("B: Intake Push");
            telemetry.addLine("X: Intake Off");
            telemetry.addLine();
            telemetry.addLine("DPad Right: Home Mode");
            telemetry.addLine();
            telemetry.addLine("Hanging: 1. DPad Up, 2. DPad Right, 3. DPad Down");
            telemetry.addLine();
            telemetry.addData("Right Motor", rDrive.getPower());
            telemetry.addData(" Left Motor", lDrive.getPower());
            telemetry.addData("        Arm", armPosition);
            telemetry.addData("      Wrist", wrist.getPosition());
            telemetry.addData("     Intake", intake.getPower());
            telemetry.update();
        }
    }
}
