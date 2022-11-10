package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class NoDriveCode extends LinearOpMode {
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
    boolean button_dpadup2_was_pressed = false;
    boolean button_dpadleft2_was_pressed = false;
    boolean button_dpadright2_was_pressed = false;
    boolean button_x2_was_pressed = false;
    boolean button_a2_was_pressed = false;
    boolean button_y2_was_pressed = false;
    boolean button_b2_was_pressed = false;
    double level;
    double claw;
    int yMod = 0;
    int xMod = 0;
    int liftpos;
    public double Level() {
        return level % 5;
    }

    public double Claw() {
        return claw % 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;
drive.mainLift.setTargetPosition(drive.mainLift.getCurrentPosition());
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up) {
                yMod = 1;
            } else if (gamepad1.dpad_down) {
                yMod = -1;
            } else {
                yMod = 0;
            }
            if (gamepad1.dpad_right) {
                xMod = 1;
            } else if (gamepad1.dpad_left) {
                xMod = -1;
            } else {
                xMod = 0;
            }

            // && drive.lift.getCurrentPosition() >= -5600
            //&& drive.lift.getCurrentPosition() <= -100
                if (button_a2_was_pressed) { // intake
                    drive.mainLift.setTargetPosition(0);
                    drive.mainLift.setPower(1);
                    drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (button_b2_was_pressed){ // low
                    drive.mainLift.setTargetPosition(-1150);
                    drive.mainLift.setPower(1);
                    drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (button_x2_was_pressed){ // mid
                    drive.mainLift.setTargetPosition(-2000);
                    drive.mainLift.setPower(1);
                    drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (button_y2_was_pressed){ // high
                    drive.mainLift.setTargetPosition(-2600);
                    drive.mainLift.setPower(1);
                    drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

//                if (gamepad1.right_bumper) {
//                    drive.lift.setPower(-1);
//                } else if (gamepad1.left_bumper) {
//                    drive.lift.setPower(1);
//                } else {
//                    drive.lift.setPower(0);
//                }
//            }

//            if (gamepad2.a && drive.mainLift.getCurrentPosition() <= -10) {
//                drive.mainLift.setPower(0.1);
//                drive.backupLift.setPower(-0.1);
//            } else if (gamepad2.y && drive.mainLift.getCurrentPosition() >= -1000) {
//                drive.mainLift.setPower(-1);
//                drive.backupLift.setPower(1);
//            } else {
//                drive.mainLift.setPower(0);
//                drive.backupLift.setPower(0);
//            }
//            if (gamepad2.a && drive.mainLift.getCurrentPosition() <= -100 && drive.mainLift.getTargetPosition() <= -100) {
//             liftpos = drive.mainLift.getTargetPosition();
//             liftpos += 40;
//             drive.mainLift.setTargetPosition(liftpos);
//                drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if (gamepad2.y && drive.mainLift.getCurrentPosition() >= -2950 && drive.mainLift.getTargetPosition() >= -2950) {
//                liftpos = drive.mainLift.getTargetPosition();
//                liftpos -= 40;
//                drive.mainLift.setTargetPosition(liftpos);
//                drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            if (Math.abs(drive.mainLift.getCurrentPosition() - drive.mainLift.getTargetPosition()) <= 5 ){
//                drive.mainLift.setPower(0);
//            } else if (drive.mainLift.isBusy()) {
//                drive.mainLift.setPower(-1);
//            }
            if (Claw() == 0) {
                drive.claw.setPosition(0.3);//0.225
                telemetry.addData("Claw:", "Closed");
            } else if (Claw() == 1) {
                drive.claw.setPosition(0.150);//0.150
                telemetry.addData("Claw:", "Open");
            }
//        if (gamepad1.dpad_down && ((yPos - drive.arm.getPosition()) <= 0.02)) {
//            drive.arm.setPosition(yPos - 0.003);
//            yPos = drive.arm.getPosition();
//        } else if (gamepad1.dpad_up && (drive.arm.getPosition() - yPos) <= 0.02) {
//            drive.arm.setPosition(yPos + 0.003);
//            yPos = drive.arm.getPosition();
//        }

            if (turntoforward && !turningtoleft && !turningtoright) {
                autoHome = true;
            }
            if (turntoleft && !autoHome && !turningtoright) {
                if (!lastwasforward) {
                    autoHome = true;
                } else {
                    turningtoleft = true;
                }
            }
            if (turntoright && !autoHome && !turningtoleft) {
                if (!lastwasforward) {
                    autoHome = true;
                } else {
                    turningtoright = true;
                }
            }
//            if (turningtoleft && lastwasforward) {
//                telemetry.addData("TT at ", "Trying to turn Left");
//                telemetry.update();
//                drive.turntable.setTargetPosition(-825);
//                drive.turntable.setPower(-0.4);
//                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if (drive.turntable.getTargetPosition() - drive.turntable.getCurrentPosition() > 10) {
//                    drive.turntable.setPower(0);
//                    turntoleft = false;
//                    lastwasleft = true;
//                    turningtoleft = false;
//                    lastwasforward = false;
//                    telemetry.addData("TT at ", "Left");
//                    telemetry.update();
//                }
//            }
//            if (turningtoright && lastwasforward) {
//                telemetry.addData("TT at ", "Trying to turn right");
//                drive.turntable.setTargetPosition(825);
//                drive.turntable.setPower(0.4);
//                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if (drive.turntable.getTargetPosition() - drive.turntable.getCurrentPosition() < 10) {
//                    drive.turntable.setPower(0);
//                    turntoright = false;
//                    lastwasright = true;
//                    lastwasforward = false;
//                    turningtoright = false;
//                    telemetry.addData("TT at ", "Right");
//                    telemetry.update();
//                }
//            }
//            if (autoHome && drive.turnlimiter.getState()) {
//                telemetry.addData("TT is ", "Trying to go Home");
//                telemetry.update();
//                if (drive.turntable.getCurrentPosition() < 0) {
//                    drive.turntable.setPower(0.3);
//                } else {
//                    drive.turntable.setPower(-0.3);
//                }
//
//            } else if (autoHome && !drive.turnlimiter.getState()) {
//                drive.turntable.setPower(0);
//                drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                turntoforward = false;
//                lastwasright = false;
//                lastwasleft = false;
//                autoHome = false;
//                lastwasforward = true;
//                telemetry.addData("TT at ", "Home");
//                telemetry.update();
//            } else {
                double tablePower = 0;
                if (!turningtoright && !turningtoleft && !autoHome && drive.turntable.getCurrentPosition() > 2000) {
                    tablePower = -gamepad2.left_trigger;
                } else if (!turningtoright && !turningtoleft && !autoHome && drive.turntable.getCurrentPosition() < -2000) {
                    tablePower = gamepad2.right_trigger;
                } else {
                    tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
                }
//                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.turntable.setPower(tablePower / 2);




/*
            // test zone for future use
            if (turntoforward && !turningtoleft && !turningtoright) {
                autoHome = true;
            }
            if (turntoleft && !autoHome && !turningtoright) {
                if (!lastwasforward) {
                    autoHome = true;
                } else {
                    turningtoleft = true;
                }
            }
            if (turntoright && !autoHome && !turningtoleft) {
                if (!lastwasforward) {
                    autoHome = true;
                } else {
                    turningtoright = true;
                }
            }
            if (turningtoleft && lastwasforward) {
                telemetry.addData("TT at ", "Trying to turn Left");
                telemetry.update();
                drive.turntable.setTargetPosition(-825);
                drive.turntable.setPower(-0.4);
                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (drive.turntable.getTargetPosition() - drive.turntable.getCurrentPosition() > 10) {
                    drive.turntable.setPower(0);
                    turntoleft = false;
                    lastwasleft = true;
                    turningtoleft = false;
                    lastwasforward = false;
                    telemetry.addData("TT at ", "Left");
                    telemetry.update();
                }
            }
            if (turningtoright && lastwasforward) {
                telemetry.addData("TT at ", "Trying to turn right");
                drive.turntable.setTargetPosition(825);
                drive.turntable.setPower(0.4);
                if(drive.turnlimiter.getState()) {
                    drive.turntable.setPower(0);
                    turntoright = false;
                    lastwasright = true;
                    lastwasforward = false;
                    turningtoright = false;
                    telemetry.addData("TT at ", "Right");
                    telemetry.update();
                }
            }
            if (autoHome && drive.turnlimiter.getState()) {
                telemetry.addData("TT is ", "Trying to go Home");
                telemetry.update();
                if (drive.turntable.getCurrentPosition() < 0) {
                    drive.turntable.setPower(0.3);
                } else {
                    drive.turntable.setPower(-0.3);
                }

            } else if (autoHome && !drive.turnlimiter.getState()) {
                drive.turntable.setPower(0);
                drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turntoforward = false;
                lastwasright = false;
                lastwasleft = false;
                autoHome = false;
                lastwasforward = true;
                telemetry.addData("TT at ", "Home");
                telemetry.update();
            } else {
                double tablePower = 0;
                if (!turningtoright && !turningtoleft && !autoHome && drive.turntable.getCurrentPosition() > 2000) {
                    tablePower = -gamepad2.left_trigger;
                } else if (!turningtoright && !turningtoleft && !autoHome && drive.turntable.getCurrentPosition() < -2000) {
                    tablePower = gamepad2.right_trigger;
                } else {
                    tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
                }
                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.turntable.setPower(tablePower / 2);
            }



 */
            // Update everything. Odometry. Etc.
            drive.update();
            if (gamepad2.dpad_down && !button_dpaddown2_was_pressed) {
                claw++;
                button_dpaddown2_was_pressed = true;
            } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
                button_dpaddown2_was_pressed = false;
            }
            if (gamepad2.dpad_up && !button_dpadup2_was_pressed) {
                turntoforward = true;
                button_dpadup2_was_pressed = true;
            } else if (!gamepad2.dpad_up && button_dpadup2_was_pressed) {
                button_dpadup2_was_pressed = false;
            }
            if (gamepad2.dpad_left && !button_dpadleft2_was_pressed) {
                turntoleft = true;
                button_dpadleft2_was_pressed = true;
            } else if (!gamepad2.dpad_left && button_dpadleft2_was_pressed) {
                button_dpadleft2_was_pressed = false;
            }
            if (gamepad2.dpad_right && !button_dpadright2_was_pressed) {
                turntoright = true;
                button_dpadright2_was_pressed = true;
            } else if (!gamepad2.dpad_right && button_dpadright2_was_pressed) {
                button_dpadright2_was_pressed = false;
            }

            if (gamepad2.x && !button_x2_was_pressed) {
                button_x2_was_pressed = true;
            } else if (!gamepad2.x && button_x2_was_pressed) {
                button_x2_was_pressed = false;
            }
            if (gamepad2.a && !button_a2_was_pressed) {
                button_a2_was_pressed = true;
            } else if (!gamepad2.a && button_a2_was_pressed) {
                button_a2_was_pressed = false;
            }
            if (gamepad2.y && !button_y2_was_pressed) {
                button_y2_was_pressed = true;
            } else if (!gamepad2.y && button_y2_was_pressed) {
                button_y2_was_pressed = false;
            }
            if (gamepad2.b && !button_b2_was_pressed) {
                button_b2_was_pressed = true;
            } else if (!gamepad2.y && button_b2_was_pressed) {
                button_b2_was_pressed = false;
            }
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Print pose to telemetry
            telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
            telemetry.addData("Lift", drive.mainLift.getCurrentPosition());
            telemetry.addData("Lift Projected", drive.mainLift.getTargetPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("TT is at home", lastwasforward);
            if (Level() == 1) {
                telemetry.addData("Level:", "Intake");
            } else if (Level() == 2) {
                telemetry.addData("Level:", "Low");
            } else if (Level() == 3) {
                telemetry.addData("Level:", "Mid (like you)");
            } else if (Level() == 4) {
                telemetry.addData("Level:", "High");
            } else {
                telemetry.addData("Manual", "Control");
            }
            telemetry.update();
        }
    }
}

