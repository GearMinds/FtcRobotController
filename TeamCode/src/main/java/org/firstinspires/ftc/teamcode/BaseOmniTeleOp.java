package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.OmniDriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class BaseOmniTeleOp extends Robot {
    OmniDriveTrain drive = new OmniDriveTrain();

    @Override
    public void setup() {}

    @Override
    public void cycle(double delta) {
        double leftPower = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        double rightPower = -gamepad1.left_stick_y + gamepad1.right_stick_x;

        double overflow = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (overflow > 1.0) {
            leftPower /= overflow;
            rightPower /= overflow;
        }

        this.drive.setLeftPower(leftPower);
        this.drive.setRightPower(rightPower);
    }
}
