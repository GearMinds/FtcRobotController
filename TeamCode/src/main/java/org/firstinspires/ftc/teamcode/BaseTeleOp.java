package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class BaseTeleOp extends Robot {
    DriveTrain drive = new DriveTrain();

    @Override
    public void setup() {
        // Might need to change the motor direction depending on how it's wired up
        this.drive.setLeftDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void cycle(double delta) {
        double leftPower = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        double rightPower = -gamepad1.left_stick_y + gamepad1.right_stick_x;

        double overflow = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (overflow > 1.0) {
            leftPower /= overflow;
            rightPower /= overflow;
        }

        drive.setLeftPower(leftPower);
        drive.setRightPower(rightPower);
    }
}
