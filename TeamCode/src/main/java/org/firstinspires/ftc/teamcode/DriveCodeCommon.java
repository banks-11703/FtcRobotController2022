package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp(name = "DriveCodeCommon", group = "Linear Opmode")
@Config
@Disabled
public class DriveCodeCommon extends LinearOpMode {

    boolean autoHome = false;
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
    boolean resettingEncoders = false;
    boolean magnetwastouched;
    boolean magnetwastouchedduringauto;
    double coneLevel;
    double claw;
    double turntimer = 0;
    int yMod = 0;
    int xMod = 0;
    int liftpos;
    int turnsensorcounter = 0;
    double autoHomeSpeed;

    public double ConeLevel() {
        return coneLevel % 5;
    }

    public double ClawToggle() {
        return claw % 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
        drive.mainLift.setTargetPosition(drive.mainLift.getCurrentPosition());
    }

    public void OdoDriving() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y + yMod,
                        -gamepad1.left_stick_x - xMod,
                        -((gamepad1.right_stick_x) / 2)
                )
        );
    }

    public void RawDriving() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.leftFront.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.5));
        drive.rightFront.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.5));
        drive.rightRear.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.5));
        drive.leftRear.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.5));

    }

    public void Toggles() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (gamepad2.dpad_down && !button_dpaddown2_was_pressed) {
            claw++;
            button_dpaddown2_was_pressed = true;
        } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
            button_dpaddown2_was_pressed = false;
        }
        if (gamepad2.dpad_up && !button_dpadup2_was_pressed) {
            autoHome = true;
            button_dpadup2_was_pressed = true;
        } else if (!gamepad2.dpad_up && button_dpadup2_was_pressed) {
            button_dpadup2_was_pressed = false;
        }
        if (gamepad2.dpad_left && !button_dpadleft2_was_pressed) {
            button_dpadleft2_was_pressed = true;
        } else if (!gamepad2.dpad_left && button_dpadleft2_was_pressed) {
            button_dpadleft2_was_pressed = false;
        }
        if (gamepad2.dpad_right && !button_dpadright2_was_pressed) {
            coneLevel++;
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
        } else if (!gamepad2.b && button_b2_was_pressed) {
            button_b2_was_pressed = false;
        }
        if (gamepad2.left_stick_button && gamepad2.right_stick_button && gamepad1.start && !resettingEncoders) {
            resettingEncoders = true;
        } else if (!(gamepad2.left_stick_button && gamepad2.right_stick_button && gamepad1.start) && resettingEncoders) {
            drive.mainLift.setPower(0);
            resettingEncoders = false;
        }
    }

    public void Lift() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (resettingEncoders) {
            drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.mainLift.setPower(-0.4);
        } else if (button_a2_was_pressed) { // intake
            drive.mainLift.setTargetPosition(0);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (button_b2_was_pressed) { // low
            drive.mainLift.setTargetPosition(1050);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (button_x2_was_pressed) { // mid
            drive.mainLift.setTargetPosition(1800);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (button_y2_was_pressed) { // high
            drive.mainLift.setTargetPosition(2500);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (button_dpadleft2_was_pressed) {
            drive.mainLift.setTargetPosition(drive.mainLift.getTargetPosition() + 25);
        }
        ConeStackLift();
    }

    public void ConeStackLift() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if ((button_dpadright2_was_pressed) && ConeLevel() == 4) {
            drive.mainLift.setTargetPosition(0);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 3) {
            drive.mainLift.setTargetPosition(38);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 2) {
            drive.mainLift.setTargetPosition(122);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 1) {
            drive.mainLift.setTargetPosition(223);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 0) {
            drive.mainLift.setTargetPosition(318);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void Claw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (ClawToggle() == 0) {
            drive.claw.setPosition(0);//0.225
            telemetry.addData("Claw:", "Open");
        } else if (ClawToggle() == 1) {
            drive.claw.setPosition(0.95);//0.150
            telemetry.addData("Claw:", "Closed");
        }
    }

    public void TurnTable() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (gamepad2.back) {
            drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (autoHome && Math.abs(drive.turntable.getCurrentPosition()) > 5) {
            telemetry.addData("TT is ", "Trying to go Home");
            drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (Math.abs(drive.turntable.getCurrentPosition()) > 200) {
                autoHomeSpeed = 1;
            } else if (Math.abs(drive.turntable.getCurrentPosition()) > 100) {
                autoHomeSpeed = 0.75;
            } else if (Math.abs(drive.turntable.getCurrentPosition()) > 5) {
                autoHomeSpeed = 0.50;
            } else {
                autoHomeSpeed = 0.25;
            }
            if (drive.turntable.getCurrentPosition() > 0) {
                drive.turntable.setPower(-autoHomeSpeed);
            } else {
                drive.turntable.setPower(autoHomeSpeed);
            }

        } else if (autoHome && Math.abs(drive.turntable.getCurrentPosition()) < 5) {
            autoHome = false;
            drive.turntable.setPower(0);
            telemetry.addData("TT at ", "Home");
        } else {
            drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double tablePower;
            tablePower = gamepad2.right_trigger - gamepad2.left_trigger;

            drive.turntable.setPower(tablePower);
        }
    }

    public void Telemetry() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Update everything. Odometry. Etc.
        drive.update();
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("turn counter", turntimer);
        telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
        telemetry.addData("Lift", drive.mainLift.getCurrentPosition());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("ConeLevel", ConeLevel() + 1);
        telemetry.addData("Turntable Position", drive.turntable.getCurrentPosition());
        telemetry.update();
    }

    public void Lights(){

    }
}