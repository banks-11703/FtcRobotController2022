package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class DriveCode extends DriveCodeCommon {
    @Override
    public void runOpMode(){
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
        RawDriving();
        Toggles();
        Lift();
        Claw();
        TurnTable();
        Lights();
        Telemetry();
            }
        }
    }

