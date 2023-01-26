package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "DriveCodeCommonNoAutoHome", group = "Linear Opmode")
@Config
@Disabled
public class DriveCodeCommonNoAutoHome extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    double maxvel = 2787.625;
    double Timestamp = 0;
    boolean autoHome = false;
    boolean atHome = false;
    boolean button_dpaddown2_was_pressed = false;
    boolean button_dpadup2_was_pressed = false;
    boolean button_dpadleft2_was_pressed = false;
    boolean button_dpadright2_was_pressed = false;
    boolean button_x2_was_pressed = false;
    boolean button_a2_was_pressed = false;
    boolean button_y2_was_pressed = false;
    boolean button_b2_was_pressed = false;
    boolean resettingEncoders = false;
    boolean shootout = false;
    boolean readytoclose;
    boolean clawrunonce = false;
    boolean turntoleft = false;
    boolean lastwasleft = false;
    boolean turningtoleft = false;
    boolean lastwasforward = false;
    boolean turntoright = false;
    boolean lastwasright = true;
    boolean turningtoright = false;
    boolean shooterEngaged;
    boolean button_a_was_pressed;
    boolean button_x_was_pressed;
    boolean firstrun = true;
    int liftLevel = 1;
    int hclaw = 1;
    int lclaw;
    int latch;
    double turntimer = 0;
    int yMod = 0;
    int xMod = 0;
    double autoHomeSpeed;
    double sliftheight = 0;
    int team;
    public double sclawh [] = {1,0.072,0.27,0.45 ,0.56};
    public double liftLevel() {
        return liftLevel % 5;
    }

    public double hClawToggle() {
        return hclaw % 2;
    }
    public double lClawToggle() {
        return lclaw % 2;
    }
    public double LatchToggle() {
        return latch % 2;
    }
    public double TimeSinceStamp() {
        return runtime.time() - Timestamp;
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void Initialization(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.slift.scaleRange(0.01,0.32);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.turntable.setTargetPosition(drive.turntable.getCurrentPosition());
        drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
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
            button_dpaddown2_was_pressed = true;
        } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
            button_dpaddown2_was_pressed = false;
        }
        if (gamepad2.dpad_up && !button_dpadup2_was_pressed) {

            button_dpadup2_was_pressed = true;
        } else if (!gamepad2.dpad_up && button_dpadup2_was_pressed) {
            button_dpadup2_was_pressed = false;
        }
        if (gamepad2.x && !button_x2_was_pressed) {
            hclaw++;
            button_x2_was_pressed = true;
        } else if (!gamepad2.x && button_x2_was_pressed) {
            button_x2_was_pressed = false;
        }
        if (gamepad1.x && !button_x_was_pressed) {
            lclaw++;
            button_x_was_pressed = true;
        } else if (!gamepad1.x && button_x_was_pressed) {
            button_x_was_pressed = false;
        }
        if (gamepad2.a && !button_a2_was_pressed) {
//            if(firstrun){firstrun = false;}
            liftLevel--;
            button_a2_was_pressed = true;
        } else if (!gamepad2.a && button_a2_was_pressed) {
            button_a2_was_pressed = false;
        }
        if (gamepad1.a && !button_a_was_pressed ) {
            latch++;
            button_a_was_pressed = true;
        } else if (!gamepad1.a && button_a_was_pressed) {
            button_a_was_pressed = false;
        }
        if (gamepad2.y && !button_y2_was_pressed) {
            autoHome = true;
            button_y2_was_pressed = true;
        } else if (!gamepad2.y && button_y2_was_pressed) {
            button_y2_was_pressed = false;
        }
        if (gamepad2.b && !button_b2_was_pressed) {
//            if(firstrun){firstrun = false;}
            liftLevel++;
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

         if (liftLevel() == 1) { // intake
            drive.mainLift.setTargetPosition(10);
            drive.mainLift.setPower(0.5);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (liftLevel() == 2  ) { // low 1050
            drive.mainLift.setTargetPosition(625);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (liftLevel() == 3) { // mid 1800
            drive.mainLift.setTargetPosition(1150);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (liftLevel() == 4) { // high
            drive.mainLift.setTargetPosition(1600);
            drive.mainLift.setPower(1);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

//        if (button_dpadleft2_was_pressed) {
//            drive.mainLift.setTargetPosition(drive.mainLift.getTargetPosition() + 25);
//        }
//        ConeStackLift();
    }

//    public void ConeStackLift() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        if ((button_dpadright2_was_pressed) && ConeLevel() == 4) {
//            drive.mainLift.setTargetPosition(0);
//            drive.mainLift.setPower(1);
//            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 3) {
//            drive.mainLift.setTargetPosition(10);
//            drive.mainLift.setPower(1);
//            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 2) {
//            drive.mainLift.setTargetPosition(82);
//            drive.mainLift.setPower(1);
//            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 1) {
//            drive.mainLift.setTargetPosition(165);
//            drive.mainLift.setPower(1);
//            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else if ((button_dpadright2_was_pressed) && ConeLevel() == 0) {
//            drive.mainLift.setTargetPosition(175);
//            drive.mainLift.setPower(1);
//            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//    }

    public void Claw() {
        if (lClawToggle() == 0) {
            SClaw(true);
        }else{
            SClaw(false);
        }
        if (hClawToggle() == 0) {
            MClaw(true);
            if (!clawrunonce){
                Timestamp = runtime.time();
                clawrunonce = true;
            }
            if (TimeSinceStamp() >= 1000){
                readytoclose = true;
            }
            telemetry.addData("Time Since Opened", TimeSinceStamp());

            telemetry.addData("Claw:", "Open");
        } else if (hClawToggle() == 1) {
            telemetry.addData("Claw:", "Closed");
            readytoclose = false;
            MClaw(false);
        }
    }

    public void TurnTable() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        if (gamepad2.back) {
//            drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

            drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double tablePower;
//            if (!autoHome && drive.turntable.getCurrentPosition() > 10 && drive.mainLift.getCurrentPosition() <= 200) {
//                tablePower = -gamepad2.left_trigger;
//            } else if (!autoHome && drive.turntable.getCurrentPosition() < -10 && drive.mainLift.getCurrentPosition() <= 1400) { //turn towards shootout
//                tablePower = gamepad2.right_trigger;
//            } else if (!autoHome && drive.turntable.getCurrentPosition() > -875 && drive.turntable.getCurrentPosition() < 10) { // shootout clearance
//                tablePower = -gamepad2.left_trigger;
//            } else if (!autoHome && drive.turntable.getCurrentPosition() < -2000) {
//                tablePower = gamepad2.right_trigger;
//            } else {
//                tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
//            }


            if(liftLevel() == 1) {
                tablePower = (gamepad2.right_trigger - gamepad2.left_trigger)/2;
            } else {
                tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
            }
            telemetry.addData("Turn Table Power", tablePower);
            drive.turntable.setPower(tablePower);


//   else {
//                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            double tablePower;
//            if (!autoHome && drive.turntable.getCurrentPosition() > 10 && drive.mainLift.getCurrentPosition() <= 200) {
//                tablePower = -gamepad2.left_trigger;
//            } else if (!autoHome && drive.turntable.getCurrentPosition() < -10 && drive.mainLift.getCurrentPosition() <= 1400) { //turn towards shootout
//                tablePower = gamepad2.right_trigger;
//            } else if (!autoHome && drive.turntable.getCurrentPosition() > -875 && drive.turntable.getCurrentPosition() < 10) { // shootout clearance
//                tablePower = -gamepad2.left_trigger;
//            } else if (!autoHome && drive.turntable.getCurrentPosition() < -2000) {
//                tablePower = gamepad2.right_trigger;
//            } else {
//                tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
//            }
//            drive.turntable.setPower(tablePower);
//            // 90 degrees for tt is 875 (less likely)
//        }
    }

    public void Telemetry() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Update everything. Odometry. Etc.
        drive.update();
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("Trying to go home", autoHome);
        telemetry.addData("Lift", drive.mainLift.getCurrentPosition());
        telemetry.addData("LiftLevel", liftLevel());
        telemetry.addData("Turntable Position", drive.turntable.getCurrentPosition());
        telemetry.addData("First Run?", firstrun);
        telemetry.addData("TT Offset", motorOffset(drive.turntable));
        telemetry.update();
    }

    public void Lights() {
        //check if you can just set cp1 and cp2 in code
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (!isStarted()) {
            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (PoseStorage.team == 0) {
            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }

    public void ShootOut() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        shooterEngaged = LatchToggle() != 0;

        if (!shooterEngaged) {
            drive.shooter.setPower(0);
            drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Latch(false);

            drive.slift.setPosition(sliftheight);
            telemetry.addData("slift",drive.slift.getPosition());
            telemetry.addData("Shooter Ticks", drive.shooter.getCurrentPosition());
        } else {
            Latch(true);
            if (gamepad1.left_bumper) {
                drive.shooter.setPower(-1);
            } else if (gamepad1.right_bumper) {
                drive.shooter.setPower(1);
            } else {
                drive.shooter.setPower(0);
            }
            if (gamepad1.dpad_up){
                sliftheight += 0.1;
            }
            if (gamepad1.dpad_down){
                sliftheight -= 0.1;
            }
        }
    }
    public void MClaw(boolean open) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open){
            drive.claw.setPosition(0.55);//0
        }else{
            SClaw(true);
            drive.claw.setPosition(0.69);//0.6
        }
    }
    public void SClaw(boolean open){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open){
            drive.sclaw.setPosition(0.05);//0.225
        }else{
            drive.sclaw.setPosition(0.29);//0.95
        }
    }
    public void Latch(boolean open){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open){
            drive.latch.setPosition(0);//0.225
        }else{
            drive.latch.setPosition(1);//0.95
        }
    }
    public void ColorSensor(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("red", drive.csensor.red());
        telemetry.addData("blue", drive.csensor.blue());
        telemetry.addData("distance", drive.csensor.getDistance(DistanceUnit.INCH));
        if(readytoclose) {
            if (PoseStorage.team == 0) {
                if (drive.csensor.blue() <= 200 && drive.csensor.red() >= 200 && drive.csensor.getDistance(DistanceUnit.INCH) <= 0.25) {
                    hclaw = 0;
                }
            } else {
                if (drive.csensor.blue() >= 200 && drive.csensor.red() <= 200 && drive.csensor.getDistance(DistanceUnit.INCH) <= 0.25) {
                    hclaw = 0;
                }
            }
        }
    }
    public int motorOffset(DcMotorEx motor){
        return Math.abs((motor.getTargetPosition())-(motor.getCurrentPosition()));

    }
}