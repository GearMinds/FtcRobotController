package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpModeClaw extends LinearOpMode {
    // Arm ticks per degree of rotation
    // comes out to around 19.792489314064724
    final double ARM_TPD = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    // constants to use for pre-defined arm positions
    // values taken from sample code
    final int ARM_INITIAL       = 0;
    final int ARM_COLLECT       = (int) (250 * ARM_TPD);
    final int ARM_CLEAR_BARRIER = (int) (225 * ARM_TPD);
    final int ARM_SCORE_SAMPLES = (int) (155 * ARM_TPD);
    final int ARM_STRAIGHT_UP   = (int) (120 * ARM_TPD);
    final int ARM_WINCH_ROBOT   = (int) (15 * ARM_TPD);

    final float HELLO_WORLD = 0.234f;

    final double CLAW_OPEN = 0.68;
    final double CLAW_CLOSED = 0.98;

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

    private Servo setupClawServo(String label) {
        Servo claw = hardwareMap.get(Servo.class, label);
        claw.setPosition(CLAW_OPEN);
        return claw;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lDrive = setupDriveMotor("l_drive", DcMotorSimple.Direction.REVERSE);
        DcMotor rDrive = setupDriveMotor("r_drive", DcMotorSimple.Direction.FORWARD);

        DcMotor slide = setupArmMotor("slide");
        DcMotor arm = setupArmMotor("arm");

        Servo claw = setupClawServo("claw");

        boolean slideExtended = false;
        boolean xButtonPressed = false;

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

            lDrive.setPower(l);
            rDrive.setPower(r);

            if (gamepad1.right_trigger > 0.5) {
                armPosition += 1;
            } else if (gamepad1.left_trigger > 0.5) {
                armPosition -= 1;
            }

            // open and close the claw
            if (gamepad1.a)
                claw.setPosition(CLAW_CLOSED);
            if (gamepad1.b)
                claw.setPosition(CLAW_OPEN);

            // extend or retract the slide
            if (gamepad1.x) {
                if (!xButtonPressed) {
                    slideExtended = !slideExtended;
                }
                xButtonPressed = true;
            } else {
                xButtonPressed = false;
            }

            // arm position mappings
            if (gamepad1.y) // Y -> Move arm to score in the basket
                armPosition = ARM_SCORE_SAMPLES;

            if (gamepad1.left_bumper) // LB -> Move arm to clear barrier
                armPosition = ARM_CLEAR_BARRIER;
            if (gamepad1.right_bumper) // RB -> Move arm to intake position and turn on intake
                armPosition = ARM_COLLECT;

            // controls for arm winching
            if (gamepad1.dpad_up) // D-Up -> prepare arm for hanging
                armPosition = ARM_STRAIGHT_UP;
            if (gamepad1.dpad_down) // D-Down -> winch robot upwards
                armPosition = ARM_WINCH_ROBOT;
            if (gamepad1.dpad_left) // D-Left -> Reset arm to original position
                armPosition = ARM_INITIAL + 75;
            if (gamepad1.dpad_right) // D-Right -> Set arm to specimen score configuration
                armPosition = ARM_SCORE_SAMPLES;

            int slidePosition = armPosition > (ARM_SCORE_SAMPLES + 5) ? -900 : -1700;

            // set target position
            arm.setTargetPosition(armPosition);
            arm.setPower(1.0); // go at full power!
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slide.setTargetPosition(slideExtended ? slidePosition : 0);
            slide.setPower(1.0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // send relevant data to driver station
            telemetry.addLine("L Analog: Forward / Backward");
            telemetry.addLine("R Analog: Left / Right");
            telemetry.addLine();
            telemetry.addLine("R Bumper: Claw grab mode");
            telemetry.addLine("L Bumper: Clear Lower Barrier");
            telemetry.addLine("Y: Score Mode");
            telemetry.addLine();
            telemetry.addLine("A: Claw open");
            telemetry.addLine("B: Claw close");
            telemetry.addLine();
            telemetry.addLine("X: Move slide in / out");
            telemetry.addLine();
            telemetry.addLine("DPad Right: Home Mode");
            telemetry.addLine();
            telemetry.addLine("Hanging: 1. DPad Up, 2. DPad Right, 3. DPad Down");
            telemetry.addLine();
            telemetry.addData("Right Motor", rDrive.getPower());
            telemetry.addData(" Left Motor", lDrive.getPower());
            telemetry.addData("   Extended", slideExtended ? "extended" : "retracted");
            telemetry.addData("      Slide", slidePosition);
            telemetry.addData("        Arm", armPosition);
            telemetry.update();
        }
    }
}