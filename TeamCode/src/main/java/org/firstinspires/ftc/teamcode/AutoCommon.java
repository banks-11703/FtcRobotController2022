package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AutoCommon", group = "Linear Opmode")
@Config
@Disabled
public class AutoCommon extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    double timeStampLift = 0;
    double timeStampShootout = 0;
    double timeStampStart = 0;
    int armTaskNum = 0;
    int shootOutTaskNum = 0;
    int turntableMod;
    boolean armDone = false;
    boolean shootoutDone = false;
    double[] coneHeights = {0,0.00,0.10,0.22,0.32,0.47};
    double[] coneHeightsClear = {0,0.15,0.55,0.72,0.91,1.0};

    boolean button_b_was_pressed = false;
    boolean button_a_was_pressed = false;
    boolean button_x_was_pressed = false;
    int team = 0;// 0 = red 1 = blue    which team we are on
    int side = 0;// 0 = left 1 = right  are we left or right
    int Mode = 0;//0 = nothing          are we just parking or otherwise?
    int autoParkPosition = 0;
    int numOfTrajs = 1;
    int yMod = 1;
    int xReflect = 1;
    double xShift = 0;
    int headingMod = 0;
    int turnMod = 1;
    double tileWidth = 23.5;
    boolean liftWait = false;
    boolean lazyShootout = false;

    public double TimeSinceStampLift() {
        return runtime.time() - timeStampLift;
    }
    public double TimeSinceStampShootout() {
        return runtime.time() - timeStampShootout;
    }
    public double TimeSinceStart() {
        return runtime.time() - timeStampStart;
    }


    double bugMultiplier = (30/29);

    int RED   = 0;
    int BLUE  = 1;
    int LEFT  = 0;
    int RIGHT = 1;


    public int Team() {
        return team % 2;
    }

    public int Side() {
        return side % 2;
    }

    public int Mode() {
        return Mode % 4;
    }

    public static double movement2x =      -34.5;
    public static double scorePosx =       -27.0;
    public static double intakeStackPosx = -65.0;
    public static double park1x =          -59.5;
    public static double park2x =          -36.5;
    public static double park3x =          -11.5;
    public static double movement2y =      13;
    public static double scorePosy =       13;
    public static double intakeStackPosy = 13;
    public static double allParky =        13;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    // UNITS ARE METERS
    double tagsize = 0.0381; // 1.5 inches

    int ID_TAG_OF_INTEREST  = 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 2; // Tag ID 2 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 3; // Tag ID 3 from the 36h11 family

    AprilTagDetection tagOfInterest = null;
    Pose2d movement2 = new Pose2d(movement2x * xReflect, movement2y * yMod, Math.toRadians(-90 * yMod));
    Pose2d scorePos = new Pose2d(scorePosx * xReflect, scorePosy * yMod, Math.toRadians(0 + headingMod));
    Pose2d intakeStackPos = new Pose2d(intakeStackPosx * xReflect, intakeStackPosy * yMod, Math.toRadians(0 + headingMod));    //Intake cone stack Position
    Pose2d park1 = new Pose2d(park1x * xReflect, allParky * yMod, Math.toRadians(0 + headingMod));
    Pose2d park2 = new Pose2d(park2x * xReflect, allParky * yMod, Math.toRadians(0 + headingMod));
    Pose2d park3 = new Pose2d(park3x * xReflect, allParky * yMod, Math.toRadians(0 + headingMod));
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initialization(){
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.slift.scaleRange(0.01,0.34);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!opModeIsActive() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        autoParkPosition = 0;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST2) {
                        autoParkPosition = 1;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST3) {
                        autoParkPosition = 2;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            //Gets inputs before init for gui
            initInputs();
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Detected: ", autoParkPosition);
            //Does Telemetry
            telemetryWhileInitialization();
            telemetry.update();
        }

        processPosition(); //Which numbers correlate to each side

    }

    public void starting(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (isStopRequested()) return;
        PoseStorage.currentPose = StartingPos();
        drive.setPoseEstimate(PoseStorage.currentPose);
        camera.stopStreaming();
        sleep(100);
        telemetry.update();
        drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        closeClaw();
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void moveToScore() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);
        if (Side() == RIGHT && Team() == BLUE) {
            Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(movement2)
                    .build();
            drive.followTrajectory(Movement1);
        } else if (Side() == LEFT && Team() == RED) {
            Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-34.5, -13, Math.toRadians(90)))
                    .build();
            drive.followTrajectory(Movement1);
        } else if (Side() == RIGHT && Team() == RED) {
            Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(34.5, -13, Math.toRadians(90)))
                    .build();
            drive.followTrajectory(Movement1);
        } else if (Side() == LEFT && Team() == BLUE) {
            Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(34.5, 13, Math.toRadians(-90)))
                    .build();
            drive.followTrajectory(Movement1);
        }
        if (Side() == LEFT) {
            drive.turn(Math.toRadians(turnMod * 90));
        } else {
            drive.turn(Math.toRadians(turnMod * 90));
        }
        if (Side() == RIGHT && Team() == BLUE) {
            Trajectory ScorePreloaded = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(scorePos)
                    .build();
            drive.followTrajectory(ScorePreloaded);
        } else if(Side() == LEFT && Team() == RED) {
            Trajectory ScorePreloaded = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-27,-13,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(ScorePreloaded);
        } else if(Side() == RIGHT && Team() == RED) {
            Trajectory ScorePreloaded = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(27,-13,Math.toRadians(180)))
                    .build();
            drive.followTrajectory(ScorePreloaded);
        } else if(Side() == LEFT && Team() == BLUE) {
            Trajectory ScorePreloaded = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(27,13,Math.toRadians(0)))
                    .build();
            drive.followTrajectory(ScorePreloaded);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }


    public void moveLift(int pos) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //raise lift
        drive.mainLift.setTargetPosition(pos);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(pos == 0) {
            drive.mainLift.setPower(0.75);
        } else {
            drive.mainLift.setPower(1.00);
        }
    }

    public void turnTable(int pos) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.turntable.setTargetPosition(pos);
        drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.turntable.setPower(1);
    }

    public void openClaw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            drive.claw.setPosition(0);//0.225
    }

    public void closeClaw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.claw.setPosition(0.6);//0.95
    }

    public void moveShootout(int pos) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.shooter.setTargetPosition(pos);//1786 is full extension
        drive.shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.shooter.setPower(1);
    }

    public void liftShooterClaw(boolean clearing,int coneNum) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if(clearing) {
            drive.slift.setPosition(coneHeightsClear[coneNum]);
        } else {
            drive.slift.setPosition(coneHeights[coneNum]);
        }
    }

    public void openShooterClaw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.sclaw.setPosition(0.05);
    }

    public void closeShooterClaw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.sclaw.setPosition(0.29);
    }

    public void openLatch() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.latch.setPosition(0);
    }

    public void closeLatch() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.latch.setPosition(1);
    }

    public void doLiftTasks() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        switch(armTaskNum) {
            case 0://Lift arm to top
                moveLift(1501);
                armTaskNum++;
                timeStampLift = runtime.time();
                break;
            case 1://Turn table to junction
                if(TimeSinceStampLift() >= 1.200) {
                    turnTable(turntableMod*900);
                    timeStampLift = runtime.time();
                    armTaskNum++;
                }
                break;
            case 2://Go down slightly
                if(TimeSinceStampLift() >= .600) {
                    moveLift(1500);
                    timeStampLift = runtime.time();
                    armTaskNum++;
                }
                break;
            case 9:
            case 16:
            case 23:
            case 30:
            case 37:
                if(TimeSinceStampLift() >= .600) {
                    moveLift(1500);
                    timeStampLift = runtime.time();
                    armTaskNum++;
                }
                break;
            case 3:
            case 10:
            case 17:
            case 24:
            case 31:
            case 38://Drop cone
                if(TimeSinceStampLift() >= .400) {
                    openClaw();
                    timeStampLift = runtime.time();
                    timeStampShootout = runtime.time();
                    armTaskNum++;
                }
                break;
            case 4:
            case 11:
            case 18:
            case 25:
            case 32://Turn table back to center and lower lift
                if(!liftWait && TimeSinceStampLift() >= .400) {
                    turnTable(0);
                    openClaw();
                    timeStampLift = runtime.time();
                    liftWait = true;
                }
                if(liftWait && TimeSinceStampLift() >= .500) {
                    moveLift(650);
                    timeStampLift = runtime.time();
                    armTaskNum++;
                    liftWait = false;
                }
                break;
            case 6:
            case 13:
            case 20:
            case 27:
            case 34://Grab cone
                if(TimeSinceStampLift() >= 0.5) {
                    closeClaw();
                    timeStampLift = runtime.time();
                    armTaskNum++;
                }
                break;
            case 7:
            case 14:
            case 21:
            case 28:
            case 35://Lift arm to top
                if(!liftWait && TimeSinceStampLift() >= .600) {
                    openShooterClaw();
                    timeStampLift = runtime.time();
                    liftWait = true;
                } else if(liftWait && TimeSinceStampLift() >= 0.1) {
                    moveLift(1501);
                    timeStampLift = runtime.time();
                    armTaskNum++;
                    liftWait = false;
                }
                break;
            case 8:
            case 15:
            case 22:
            case 29:
            case 36://Turn table to junction
                if(TimeSinceStampLift() >= 1.300) {
                    turnTable(turntableMod*900);
                    armTaskNum++;
                }
                break;
            case 5:
                if(!liftWait &&  !drive.shooter.isBusy() && shootOutTaskNum >= 10) {
                    timeStampLift = runtime.time();
                    liftWait = true;
                }
                if(liftWait && TimeSinceStampLift() >= .8) {
                    moveLift(401);
//                    closeClaw();
                    timeStampLift = runtime.time();
                    armTaskNum++;
                    liftWait = false;
                }
                break;
            case 12:
                if(!liftWait &&  !drive.shooter.isBusy() && shootOutTaskNum >= 20) {
                    timeStampLift = runtime.time();
                    liftWait = true;
                }
                if(liftWait && TimeSinceStampLift() >= .8) {
                    moveLift(401);
//                    closeClaw();
                    timeStampLift = runtime.time();
                    armTaskNum++;
                    liftWait = false;
                }
                break;
            case 19:
                if(!liftWait &&  !drive.shooter.isBusy() && shootOutTaskNum >= 30) {
                    timeStampLift = runtime.time();
                    liftWait = true;
                }
                if(liftWait && TimeSinceStampLift() >= .8) {
                    moveLift(401);
//                    closeClaw();
                    timeStampLift = runtime.time();
                    armTaskNum++;
                    liftWait = false;
                }
                break;
            case 26:
                if(!liftWait &&  !drive.shooter.isBusy() && shootOutTaskNum >= 40) {
                    timeStampLift = runtime.time();
                    liftWait = true;
                }
                if(liftWait && TimeSinceStampLift() >= .8) {
                    moveLift(401);
//                    closeClaw();
                    timeStampLift = runtime.time();
                    armTaskNum++;
                    liftWait = false;
                }
                break;
            case 33://move lift to bottom
                if(TimeSinceStampLift() >= .5 && shootOutTaskNum >= 50) {
                    moveLift(401);
//                    closeClaw();
                    timeStampLift = runtime.time();
                    armTaskNum++;
                }
                break;
            case 39://Turn table back to center and lower arm
                if(TimeSinceStampLift() >= .150) {
                    turnTable(0);
                    moveLift(0);
                    armTaskNum++;
                }
                break;
            case 40://End loop and park
                armDone=true;
                break;
        }
    }

    public void doShootoutTasks() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        switch(shootOutTaskNum) {
            case 0:
                if(!lazyShootout) {
                    openLatch();
                    timeStampShootout = runtime.time();
                    lazyShootout = true;
                }
                if(lazyShootout && TimeSinceStampShootout() >= 0.6) {
                    moveShootout(1100);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                    lazyShootout = false;
                }
                break;
            case 1:
                liftShooterClaw(false,5);
                openShooterClaw();
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 11:
                liftShooterClaw(false,4);
                openShooterClaw();
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 21:
                liftShooterClaw(false,3);
                openShooterClaw();
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 31:
                liftShooterClaw(false,2);
                openShooterClaw();
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 41:
                liftShooterClaw(false,1);
                openShooterClaw();
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 2:
                if(TimeSinceStampShootout() >= 1.0 && armTaskNum > 3) {
                    moveShootout(1700);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 12:
                if(TimeSinceStampShootout() >= 1.0 && armTaskNum > 10) {
                    moveShootout(1700);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 22:
                if(TimeSinceStampShootout() >= 1.0 && armTaskNum > 17) {
                    moveShootout(1700);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 32:
                if(TimeSinceStampShootout() >= 1.0 && armTaskNum > 24) {
                    moveShootout(1700);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 42:
                if(TimeSinceStampShootout() >= 1.0 && armTaskNum > 31) {
                    moveShootout(1700);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 3:
            case 13:
            case 23:
            case 33:
            case 43:
                if(TimeSinceStampShootout() >= 0.4) {
                    closeShooterClaw();
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                }
                break;
            case 4:
            case 14:
            case 24:
            case 34:
            case 44:
                    shootOutTaskNum++;
                break;
            case 5:
                liftShooterClaw(true,5);
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 15:
                liftShooterClaw(true,4);
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 25:
                liftShooterClaw(true,3);
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 35:
                liftShooterClaw(true,2);
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 45:
                liftShooterClaw(true,1);
                timeStampShootout = runtime.time();
                shootOutTaskNum++;
                break;
            case 6:
            case 16:
            case 26:
            case 36:
            case 46:
                if(!lazyShootout && TimeSinceStampShootout() >= 1.50) {
                    moveShootout(15);
                    timeStampShootout = runtime.time();
                    lazyShootout = true;
                }
                if(lazyShootout && TimeSinceStampShootout() >= 0.5) {
                    drive.slift.setPosition(0.90);
                    timeStampShootout = runtime.time();
                    shootOutTaskNum++;
                    lazyShootout = false;
                }
                break;
            case 7:
            case 17:
            case 27:
            case 37:
            case 47:
                shootOutTaskNum++;
                break;
            case 8://extra in case we need more
            case 18:
            case 28:
            case 38:
            case 48:
                shootOutTaskNum++;
                break;
            case 9://extra in case we need more
            case 19:
            case 29:
            case 39:
            case 49:
                shootOutTaskNum++;
                break;
            case 10:
                if(!drive.shooter.isBusy() && armTaskNum > 7) {
                    moveShootout(1900);
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 20:
                if(!drive.shooter.isBusy() && armTaskNum > 14) {
                    moveShootout(1900);
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 30:
                if(!drive.shooter.isBusy() && armTaskNum > 21) {
                    moveShootout(1900);
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 40:
                if(!drive.shooter.isBusy() && armTaskNum > 28) {
                    moveShootout(1900);
                    shootOutTaskNum++;
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 500) {
                    drive.shooter.setPower(0.5);
                } else if(Math.abs(drive.shooter.getTargetPosition()-drive.shooter.getCurrentPosition()) <= 250) {
                    drive.shooter.setPower(0.25);
                }
                break;
            case 50:
                shootoutDone = true;
                break;
        }
    }


    public Pose2d StartingPos() {
        double x, y, a;
        if (Team() == RED) {
            y = -63;
            a = 90;
            if (Side() == RIGHT) {
                x = 34.5;//23.25
            } else {
                x = -34.5;//23.25
            }
        } else {
            y = 63;
            a = -90;
            if (Side() == RIGHT) {
                x = -34.5;//23.25
            } else {
                x = 34.5;
            }
        }

        return new Pose2d(x, y, Math.toRadians(a));
    }

    public void telemetryWhileInitialization() {
        switch (Team()) {
            case (0):
                telemetry.addData("Team", "Red");
                switch (Side()) {
                    case (0):
                        telemetry.addData("Side", "Left");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Shootout Cycle");
                                break;
                        }
                        break;
                    case (1):
                        telemetry.addData("Side", "Right");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Shootout Cycle");
                                break;
                        }
                        break;
                }
                break;
            case (1):
                telemetry.addData("Team", "Blue");
                switch (Side()) {
                    case (0):
                        telemetry.addData("Side", "Left");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Shootout Cycle");
                                break;
                        }
                        break;
                    case (1):
                        telemetry.addData("Side", "Right");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Shootout Cycle");
                                break;
                        }
                        break;
                }
                break;
        }


    }

    public void initInputs() {
        if (gamepad1.b && !button_b_was_pressed) {
            team++;
            button_b_was_pressed = true;
        } else if (!gamepad1.b && button_b_was_pressed) {
            button_b_was_pressed = false;
        }
        if (gamepad1.a && !button_a_was_pressed) {
            side++;
            button_a_was_pressed = true;
        } else if (!gamepad1.a && button_a_was_pressed) {
            button_a_was_pressed = false;
        }
        if (gamepad1.x && !button_x_was_pressed) {
            Mode++;
            button_x_was_pressed = true;
        } else if (!gamepad1.x && button_x_was_pressed) {
            button_x_was_pressed = false;
        }
    }

    public void processPosition() {
        if (team % 2 == BLUE) {
            yMod = 1;
        } else {
            yMod = -1;
        }
        if ((team % 2 == RED && side % 2 == LEFT) || (team % 2 == BLUE && side % 2 == RIGHT)) {
            xReflect = 1;
            xShift = 0;
            headingMod = 180;
        } else if ((team % 2 == 0 && side % 2 == 1) || (team % 2 == 1 && side % 2 == 0)) {
            xReflect = -1;
            xShift = 70.5;
            headingMod = 0;
        }
        if (side % 2 == 0) {
            turnMod = -1;
        } else {
            turnMod = 1;
        }
    }

    public void doNothing() {

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}