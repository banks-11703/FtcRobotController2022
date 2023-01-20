package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class ResetEncoders extends LinearOpMode {

    double turntimer;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
        if (isStopRequested()) return;
        drive.mainLift.setTargetPosition(drive.mainLift.getCurrentPosition());
        turntimer = 0;
        while (opModeIsActive() && !isStopRequested()) {

                drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.mainLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Read pose
                Pose2d poseEstimate = drive.getPoseEstimate();
                // Print pose to telemetry
                telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
                telemetry.addData("Lift", drive.mainLift.getCurrentPosition());
                telemetry.addData("Turntable Position", drive.turntable.getCurrentPosition());

//            if (Level() == 1) {
//                telemetry.addData("Level:", "Intake");
//            } else if (Level() == 2) {
//                telemetry.addData("Level:", "Low");
//            } else if (Level() == 3) {
//                telemetry.addData("Level:", "Mid (like you)");
//            } else if (Level() == 4) {
//                telemetry.addData("Level:", "High (as a kite)");
//            } else {
//                telemetry.addData("Manual", "Control");
//            }
                telemetry.update();
            }
        }
    }

