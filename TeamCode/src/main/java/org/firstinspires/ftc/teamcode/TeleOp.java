package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class TeleOp extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
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
        telemetry.addData("- Flywheel:     ", launcher.flywheel.getPower());
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
