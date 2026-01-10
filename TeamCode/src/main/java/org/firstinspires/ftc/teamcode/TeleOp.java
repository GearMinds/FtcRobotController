package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class TeleOp extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    boolean launching = false;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(this);
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

        // A - Toggle the flywheel on or off
        if (gamepad1.aWasPressed()) {
            launcher.toggleFlywheel();
        }

        double flywheelVelocity = ((DcMotorEx)launcher.flywheel).getVelocity();

        // B - Begin feeding balls
        // X (Hold) - Reverse feed balls
        if (gamepad1.b) {
            // Prefer the B button over X to start feeding the balls
            launcher.feed();
        } else if (gamepad1.x) {
            // Holding X will reverse balls out of the feeder if desired
            launcher.backFeed();
        } else {
            launcher.stopFeeder();
        }

        if (gamepad1.right_trigger > 0.5) {
            launching = true;
            if (launcher.isSpinning()) {
                // Start shooting the ball if we're already spinning
                launcher.feedIfReady();
            } else {
                launcher.spinFlywheel();
            }
        } else if (launching) {
            launching = false;
            launcher.stop();
        }

        // Apply breaking
        driveTrain.maxPower = 1.0 - gamepad1.left_trigger;

        // Set the drive train power
        driveTrain.setPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        // Send data to the driver station on each cycle
        telemetry.addLine("Controls:");
        telemetry.addLine("Press A - Toggle flywheel on/off");
        telemetry.addLine("Hold B  - Feed ball into flywheel");
        telemetry.addLine("Hold X  - Unfeed ball from flywheel");
        telemetry.addLine("Hold LT - Brake, limits maximum speed of robot");
        telemetry.addLine();

        telemetry.addLine("Launcher assembly power readings:");
        telemetry.addData("- Flywheel p: ", launcher.flywheel.getPower());
        telemetry.addData("- Flywheel v: ", flywheelVelocity);
        telemetry.addLine("- Feeder:");
        telemetry.addData("-- L:  ", launcher.leftFeeder.getPower());
        telemetry.addData("-- R: ", launcher.rightFeeder.getPower());
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
