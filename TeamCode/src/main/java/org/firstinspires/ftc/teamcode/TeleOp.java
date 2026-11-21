package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class TeleOp extends Robot {
    DriveTrain driveTrain;
    DcMotor intakeMotor;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        intakeMotor = this.hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void cycle(double delta) throws InterruptedException {

        // Get the desired direction from the control sticks
        double translateX = -gamepad1.left_stick_y;
        double translateZ = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Calculate what the power of each motor should be by applying the following transformations
        double leftFrontPower = translateX + translateZ + rotate;
        double rightFrontPower = translateX - translateZ - rotate;
        double leftBackPower = translateX - translateZ + rotate;
        double rightBackPower = translateX + translateZ - rotate;



        if (gamepad1.right_trigger > 0) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.left_trigger > 0) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        // Set the drive train power
        driveTrain.setPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        // Send data to the driver station on each cycle
        telemetry.addLine("Controls: TODO");
        telemetry.addLine();
        telemetry.addLine("Drivetrain power readings:");
        telemetry.addData("- Max Power: ", driveTrain.maxPower);
        telemetry.addLine("- Front:");
        telemetry.addData("-- L: ", driveTrain.frontLeft.getPower());
        telemetry.addData("-- R: ", driveTrain.frontRight.getPower());
        telemetry.addLine("- Back:");
        telemetry.addData("-- L: ", driveTrain.backLeft.getPower());
        telemetry.addData("-- R: ", driveTrain.backRight.getPower());
        telemetry.update();
    }
}
