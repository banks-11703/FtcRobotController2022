package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    double timeStampScore = 0;
    double timeStampShootout = 0;

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

    public double TimeSinceStampScore() {
        return runtime.time() - timeStampScore;
    }
    public double TimeSinceStampShootout() {
        return runtime.time() - timeStampShootout;
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
    public static double scorePosx =       -28.0;
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
    Pose2d scorePos = new Pose2d(scorePosx * xReflect, scorePosy * yMod, Math.toRadians(180 + headingMod));
    Pose2d intakeStackPos = new Pose2d(intakeStackPosx * xReflect, intakeStackPosy * yMod, Math.toRadians(180 + headingMod));    //Intake cone stack Position
    Pose2d park1 = new Pose2d(park1x * xReflect, allParky * yMod, Math.toRadians(180 + headingMod));
    Pose2d park2 = new Pose2d(park2x * xReflect, allParky * yMod, Math.toRadians(180 + headingMod));
    Pose2d park3 = new Pose2d(park3x * xReflect, allParky * yMod, Math.toRadians(180 + headingMod));
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void initialization(){
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field

        drive.setPoseEstimate(StartingPos());

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
            drive.setPoseEstimate(StartingPos());
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
        drive.setPoseEstimate(StartingPos());
        camera.stopStreaming();
        sleep(100);
        telemetry.update();
        drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.turntable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.claw.setPosition(0.95);
    }

    public void movetoscore(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(movement2)
                .build();
        drive.followTrajectory(Movement1);
        if (Side() == LEFT) {
            drive.turn(Math.toRadians(turnMod * 90));
        } else {
            drive.turn(Math.toRadians(turnMod * 90));
        }
        Trajectory ScorePreloaded = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(scorePos)
                .build();
        drive.followTrajectory(ScorePreloaded);
    }


    public void moveLift(int pos) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //raise lift
        drive.mainLift.setTargetPosition(pos);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.mainLift.setPower(1);
    }

    public void turnTable(int pos) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.turntable.setTargetPosition(pos);
        drive.turntable.setPower(1);
        drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void openClaw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.claw.setPosition(0);
    }

    public void closeClaw() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.claw.setPosition(0.95);
    }

    public void extendShootout() {

    }

    public void retractShootout() {

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

    double calculateSmoothishness(int currentPos, int targetPose){
        if(Math.abs(currentPos - targetPose) > 200) {
            return 1;
        } else if(Math.abs(currentPos - targetPose) > 100) {
            return 0.75;
        } else {
            return 0.50;
        }
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