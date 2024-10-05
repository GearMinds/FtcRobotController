package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Test Program", group = "Test")
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo0 = hardwareMap.get(Servo.class, "s0");
        Servo servo1 = hardwareMap.get(Servo.class, "s1");
        Servo servo2 = hardwareMap.get(Servo.class, "s2");

        double position = 0.5;
        double increment = 0.01;

        waitForStart();

        while (opModeIsActive()) {

            if (position >= 1.0 || position <= 0.0)
                increment *= -1.0;

            position += increment;

            servo0.setPosition(position);
            servo1.setPosition(position);
            servo2.setPosition(position);

        }
    }
}
