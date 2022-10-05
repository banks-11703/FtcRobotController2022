package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class DriveCode extends LinearOpMode {
    boolean autoHome;
    boolean turntoforward = false;
    boolean turntoright = false;
    boolean turntoleft = false;
    boolean lastwasforward = false;
    boolean lastwasright = false;
    boolean lastwasleft = false;
    boolean turningtoleft = false;
    boolean turningtoright = false;
    public static double ServoTesting = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double yPos = 0;
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            // && drive.lift.getCurrentPosition() >= -5600
            //&& drive.lift.getCurrentPosition() <= -100
        if (gamepad1.a){
            drive.lift.setPower(-1);
        }else if(gamepad1.b){
                drive.lift.setPower(1);
        }else{
            drive.lift.setPower(0);
        }
        if (gamepad1.y){
            drive.claw.setPosition(7);//0.225
        }else if (gamepad1.x){
            drive.claw.setPosition(0);//0.150
        }
        if (gamepad1.dpad_down && ((yPos - drive.arm.getPosition()) <= 0.02)) {
                drive.arm.setPosition(yPos - 0.003);
                yPos = drive.arm.getPosition();
        } else if (gamepad1.dpad_up && (drive.arm.getPosition() - yPos) <= 0.02) {
                drive.arm.setPosition(yPos + 0.003);
                yPos = drive.arm.getPosition();
        }

            if(turntoforward){
                autoHome = true;
            }
            if (turntoleft){
                turningtoleft = true;
                if (!lastwasforward){
                    autoHome = true;
                }
            }
            if (turntoright){
                turningtoright = true;
                if (!lastwasforward){
                    autoHome = true;
                }
            }
            if (turningtoleft && lastwasforward){
                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.turntable.setTargetPosition(-500);
                drive.turntable.setPower(0.1);
                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turntable.setPower(0);
                lastwasleft = true;
            }
            if (turningtoright && lastwasforward){
                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.turntable.setTargetPosition(500);
                drive.turntable.setPower(0.1);
                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turntable.setPower(0);
                lastwasright = true;
            }
            gamepad2.dpad_up = turntoforward;
            gamepad2.dpad_left = turntoleft;
            gamepad2.dpad_right = turntoright;

            if (autoHome && drive.turnlimiter.getState() ){
                if (lastwasleft) {
                    drive.turntable.setPower(0.1);
                }
                if (lastwasright){
                    drive.turntable.setPower(-0.1);
                }
            } else if (autoHome && !drive.turnlimiter.getState()){
                drive.turntable.setPower(0);
                drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lastwasright = false;
                lastwasleft = false;
                lastwasforward = true;
            } else{
                drive.turntable.setPower((gamepad1.left_trigger - gamepad1.right_trigger)/5);
            }
            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Print pose to telemetry
            telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
            telemetry.addData("Lift",drive.lift.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

}