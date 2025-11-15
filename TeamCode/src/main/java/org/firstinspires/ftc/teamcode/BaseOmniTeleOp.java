package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import org.firstinspires.ftc.teamcode.lib.OmniDriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Robot")
public class BaseOmniTeleOp extends Robot {
    OmniDriveTrain drive;
    DcMotor spinner;

    @Override
    public void setup() {
        drive = new OmniDriveTrain(hardwareMap);
        spinner = hardwareMap.get(DcMotor.class, "spinner");
    }

    @Override
    public void cycle(double delta) {
        double translateX = -gamepad1.left_stick_y;
        double translateZ = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean spinning = gamepad1.a;

        double leftFrontPower = translateX + translateZ + rotate;
        double rightFrontPower = translateX - translateZ - rotate;
        double leftBackPower = translateX - translateZ + rotate;
        double rightBackPower = translateX + translateZ - rotate;

        drive.frontLeft.setPower(leftFrontPower);
        drive.backLeft.setPower(leftBackPower);
        drive.frontRight.setPower(rightFrontPower);
        drive.backRight.setPower(rightBackPower);

        spinner.setPower(spinning ? 100 : 0);
    }
}
