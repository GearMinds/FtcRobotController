package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.OmniDriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class TeleOp extends Robot {
    OmniDriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() {
        driveTrain = new OmniDriveTrain(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void cycle(double delta) {

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
            launcher.unfeed();
        } else {
            launcher.stopFeeder();
        }

        // Soft holding the left trigger will slow the robot down 1/2
        // Hard holding the left trigger will slow the robot down 1/3
        if (gamepad1.left_trigger > 0.0) {
            driveTrain.maxPower = gamepad1.left_trigger;
        } else {
            // Not holding the trigger will set it back to 100%
            driveTrain.maxPower = 1.0;
        }

        // Set the drive train power
        driveTrain.setPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        telemetry.addLine("Controls:");
        telemetry.addLine("Press A - Toggle flywheel on/off");
        telemetry.addLine("Hold B  - Feed ball into flywheel");
        telemetry.addLine("Hold X  - Unfeed ball from flywheel");
        telemetry.addLine("Hold LT - Limit the top speed of the drive train");
        telemetry.addLine();

        telemetry.addLine("Launcher assembly power readings:");
        telemetry.addData("Flywheel:     ", launcher.flywheel.getPower());
        telemetry.addData("Left feeder:  ", launcher.leftFeeder.getPower());
        telemetry.addData("Right feeder: ", launcher.rightFeeder.getPower());
        telemetry.addLine();

        telemetry.addLine("Drivetrain power readings:");
        telemetry.addData("Left front:  ", leftFrontPower);
        telemetry.addData("Right front: ", rightFrontPower);
        telemetry.addData("Left back:   ", leftBackPower);
        telemetry.addData("Right back:  ", rightBackPower);
        telemetry.update();
    }
}
