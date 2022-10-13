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
    boolean button_dpaddown2_was_pressed = false;
    boolean button_x2_was_pressed = false;
    double level;
    double claw;
    int yMod = 0;
    int xMod = 0;
    public double Level() {
        return level % 5;
    }
    public double Claw(){
        return claw % 2;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double yPos = 0;
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if(gamepad1.dpad_up) {
                yMod = 1;
            }else if (gamepad1.dpad_down) {
                yMod = -1;
            }else {
                yMod = 0;
            }
            if(gamepad1.dpad_right) {
                xMod = 1;
            }else if (gamepad1.dpad_left) {
                xMod = -1;
            }else {
                xMod = 0;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y+yMod,
                            -gamepad1.left_stick_x-xMod,
                            -gamepad1.right_stick_x
                    )
            );
            // && drive.lift.getCurrentPosition() >= -5600
            //&& drive.lift.getCurrentPosition() <= -100
//            if (Level() == 1) {
//                if (button_a_was_pressed) {
//                    drive.lift.setTargetPosition(200);
//                    drive.lift.setPower(0.5);
//                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    drive.lift.setPower(0);
//                } else {
//                    drive.lift.setPower(0);
//                }
//            } else if (Level() == 4) {
//
//                if (gamepad1.right_bumper) {
//                    drive.lift.setPower(-1);
//                } else if (gamepad1.left_bumper) {
//                    drive.lift.setPower(1);
//                } else {
//                    drive.lift.setPower(0);
//                }
//            }

            if (gamepad2.a) {
                drive.lift.setPower(-1);
            } else if (gamepad2.y) {
                drive.lift.setPower(1);
            } else {
                drive.lift.setPower(0);
            }
            if (Claw() == 0) {
                drive.claw.setPosition(0.3);//0.225
                telemetry.addData("Claw:","Closed");
            } else if (Claw() == 1) {
                drive.claw.setPosition(0.150);//0.150
                telemetry.addData("Claw:","Open");
            }
//        if (gamepad1.dpad_down && ((yPos - drive.arm.getPosition()) <= 0.02)) {
//            drive.arm.setPosition(yPos - 0.003);
//            yPos = drive.arm.getPosition();
//        } else if (gamepad1.dpad_up && (drive.arm.getPosition() - yPos) <= 0.02) {
//            drive.arm.setPosition(yPos + 0.003);
//            yPos = drive.arm.getPosition();
//        }

            if (turntoforward) {
                autoHome = true;
            }
            if (turntoleft) {
                turningtoleft = true;
                if (!lastwasforward) {
                    autoHome = true;
                }
            }
            if (turntoright) {
                turningtoright = true;
                if (!lastwasforward) {
                    autoHome = true;
                }
            }
            if (turningtoleft && lastwasforward) {
                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.turntable.setTargetPosition(-500);
                drive.turntable.setPower(0.1);
                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.turntable.setPower(0);
                lastwasleft = true;
            }
            if (turningtoright && lastwasforward) {
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

            if (autoHome && drive.turnlimiter.getState()) {
                if (lastwasleft) {
                    drive.turntable.setPower(0.1);
                }
                if (lastwasright) {
                    drive.turntable.setPower(-0.1);
                }
            } else if (autoHome && !drive.turnlimiter.getState()) {
                drive.turntable.setPower(0);
                drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lastwasright = false;
                lastwasleft = false;
                lastwasforward = true;
            } else {
                drive.turntable.setPower((gamepad2.right_trigger - gamepad2.left_trigger) / 2);
            }
            // Update everything. Odometry. Etc.
            drive.update();
//            if (gamepad2.dpad_down && !button_dpaddown2_was_pressed) {
//                level++;
//                button_dpaddown2_was_pressed = true;
//            } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
//                button_dpaddown2_was_pressed = false;
//            }
            if (gamepad2.x && !button_x2_was_pressed) {
                claw++;
                button_x2_was_pressed = true;
            } else if (!gamepad2.x && button_x2_was_pressed) {
                button_x2_was_pressed = false;
            }
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Print pose to telemetry
            telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
            telemetry.addData("TurnTable", drive.turntable.getCurrentPosition());
            telemetry.addData("Lift", drive.lift.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            if (Level() == 1) {
                telemetry.addData("Level:", "Intake");
            } else if (Level() == 2) {
                telemetry.addData("Level:", "Low");
            } else if (Level() == 3) {
                telemetry.addData("Level:", "Mid (like you)");
            } else if (Level() == 4) {
                telemetry.addData("Level:", "High (as a kite)");
            } else {
                telemetry.addData("Manual", "Control");
            }
            telemetry.update();
        }
    }
}

