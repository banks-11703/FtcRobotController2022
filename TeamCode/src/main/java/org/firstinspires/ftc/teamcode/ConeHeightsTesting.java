package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class ConeHeightsTesting extends DriveCodeCommon {
    boolean closed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            if(gamepad2.a) {
                drive.slift.setPosition(0);
            } else if(gamepad2.b) {
                drive.slift.setPosition(0);
            }
            telemetry.addData("slift",drive.slift.getPosition());
            telemetry.update();

            if(button_x2_was_pressed && closed) {
                SClaw(true);
                closed = false;
            } else if(button_x2_was_pressed && !closed) {
                SClaw(false);
                closed = true;
            }

            if (gamepad2.x && !button_x2_was_pressed) {
                button_x2_was_pressed = true;
            } else if (!gamepad2.x && button_x2_was_pressed) {
                button_x2_was_pressed = false;
            }
        }
    }
}