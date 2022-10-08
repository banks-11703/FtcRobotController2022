package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Auto extends CameraTesting {
    boolean button_b_was_pressed = false;
    boolean button_a_was_pressed = false;
    boolean button_x_was_pressed = false;
    int team = 0;// 0 = red 1 = blue    which team we are on
    int side = 0;// 0 = left 1 = right  are we left or right
    int mode = 0;//0 = nothing          are we just parking or otherwise?
    int autoParkPosition = 0;
    int numOfTrajs = 1;
    public int Team() {
        return team % 2;
    }

    public int Side() {
        return side % 2;
    }

    public int Mode() {
        return mode % 3;
    }

    @Override
    public void runOpMode() {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(10, 15, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new SleeveOrientationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        while (!opModeIsActive()) {


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
                    mode++;
                    button_x_was_pressed = true;
                } else if (!gamepad1.x && button_x_was_pressed) {
                    button_x_was_pressed = false;
                }
                switch (Team()){
                    case (0):
                        telemetry.addData("Team", "Red");
                        telemetry.update();
                        switch (Side()){
                            case(0):
                                telemetry.addData("Side", "Left");
                                telemetry.update();
                                switch(Mode()){
                                    case(0):
                                        telemetry.addData("Mode", "Nothing");
                                        telemetry.update();
                                        break;
                                    case(1):
                                        telemetry.addData("Mode", "Cycle");
                                        telemetry.update();
                                        break;
                                    case(2):
                                        telemetry.addData("Mode", "Park Only");
                                        telemetry.update();
                                        break;
                                }
                                break;
                            case(1):
                                telemetry.addData("Side", "Right");
                                telemetry.update();
                                switch(Mode()){
                                    case(0):
                                        telemetry.addData("Mode", "Nothing");
                                        telemetry.update();
                                        break;
                                    case(1):
                                        telemetry.addData("Mode", "Cycle");
                                        telemetry.update();
                                        break;
                                    case(2):
                                        telemetry.addData("Mode", "Park Only");
                                        telemetry.update();
                                        break;
                                }
                                break;
                        }
                        break;
                    case (1):
                        telemetry.addData("Team", "Blue");
                        telemetry.update();
                        switch (Side()){
                            case(0):
                                telemetry.addData("Side", "Left");
                                telemetry.update();
                                switch(Mode()){
                                    case(0):
                                        telemetry.addData("Mode", "Nothing");
                                        telemetry.update();
                                        break;
                                    case(1):
                                        telemetry.addData("Mode", "Cycle");
                                        telemetry.update();
                                        break;
                                    case(2):
                                        telemetry.addData("Mode", "Park Only");
                                        telemetry.update();
                                        break;
                                }
                                break;
                            case(1):
                                telemetry.addData("Side", "Right");
                                telemetry.update();
                                switch(Mode()){
                                    case(0):
                                        telemetry.addData("Mode", "Nothing");
                                        telemetry.update();
                                        break;
                                    case(1):
                                        telemetry.addData("Mode", "Cycle");
                                        telemetry.update();
                                        break;
                                    case(2):
                                        telemetry.addData("Mode", "Park Only");
                                        telemetry.update();
                                        break;
                                }
                                break;
                        }
                        break;
                }


                autoParkPosition = pipeline.getAnalysis();




            if (isStopRequested()) return;
            drive.setPoseEstimate(StartingPos());
            autoParkPosition = pipeline.getAnalysis();
            webcam.stopStreaming();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            sleep(100);
            telemetry.update();

            /*
            switch (Team()){
                case (0):
                    telemetry.addData("Team", "Red");
                    telemetry.update();
                    switch (Side()){
                        case(0):
                            telemetry.addData("Side", "Left");
                            telemetry.update();
                            switch(Mode()){
                                case 0:
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case 1:
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case 2:
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                        case(1):
                            telemetry.addData("Side", "Right");
                            telemetry.update();
                            switch(Mode()){
                                case 0:
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case 1:
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case 2:
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                    }
                    break;
                case (1):
                    telemetry.addData("Team", "Blue");
                    telemetry.update();
                    switch (Side()){
                        case(0):
                            telemetry.addData("Side", "Left");
                            telemetry.update();
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                        case(1):
                            telemetry.addData("Side", "Right");
                            telemetry.update();
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                    }
                    break;
            }*/


            // Transfer the current pose to PoseStorage so we can use it in TeleOp
            Pose2d movement1 = new Pose2d(12,60,Math.toRadians(90));
            Pose2d movement2 = new Pose2d(12,12,Math.toRadians(180));
            Pose2d scorePos = new Pose2d(24,12,Math.toRadians(180));    //Score Position
            Pose2d intakeStackPos = new Pose2d(62,12,Math.toRadians(180));    //Intake cone stack Position
            Pose2d park1 = new Pose2d(60,12,Math.toRadians(90));
            Pose2d park2 = new Pose2d(36,12,Math.toRadians(90));
            Pose2d park3 = new Pose2d(12,12,Math.toRadians(90));
            Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(movement1)
                    .build();
            Trajectory Movement2 = drive.trajectoryBuilder(Movement1.end())
                    .lineToLinearHeading(movement2)
                    .build();
            Trajectory ScorePos1 = drive.trajectoryBuilder(Movement2.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Trajectory IntakeStackPos1 = drive.trajectoryBuilder(ScorePos1.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            Trajectory ScorePos2 = drive.trajectoryBuilder(IntakeStackPos1.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Trajectory IntakeStackPos2 = drive.trajectoryBuilder(ScorePos2.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            Trajectory ScorePos3 = drive.trajectoryBuilder(IntakeStackPos2.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Trajectory IntakeStackPos3 = drive.trajectoryBuilder(ScorePos3.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            Trajectory ScorePos4 = drive.trajectoryBuilder(IntakeStackPos3.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Trajectory IntakeStackPos4 = drive.trajectoryBuilder(ScorePos4.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            Trajectory ScorePos5 = drive.trajectoryBuilder(IntakeStackPos4.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Trajectory IntakeStackPos5 = drive.trajectoryBuilder(ScorePos5.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            Trajectory ScorePos6 = drive.trajectoryBuilder(IntakeStackPos5.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Trajectory Park1 = drive.trajectoryBuilder(ScorePos6.end())
                    .lineToLinearHeading(park1)
                    .build();
            Trajectory Park2 = drive.trajectoryBuilder(ScorePos6.end())
                    .lineToLinearHeading(park2)
                    .build();
            Trajectory Park3 = drive.trajectoryBuilder(ScorePos6.end())
                    .lineToLinearHeading(park3)
                    .build();
        }


        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d()).build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(new Pose2d()).build();
        Trajectory myTrajectory3 = drive.trajectoryBuilder(new Pose2d()).build();

        switch(autoParkPosition){
            case 1:
                numOfTrajs = 3;
                myTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(24)
                        .build();
                myTrajectory2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(50.6)
                        .build();
                myTrajectory3 = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(24)
                        .build();
                waitForStart();
                if(numOfTrajs == 1) {
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                } else if(numOfTrajs == 2){
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.followTrajectory(myTrajectory2);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                }else if(numOfTrajs == 3){
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.followTrajectory(myTrajectory2);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.followTrajectory(myTrajectory3);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                }
                if(isStopRequested())return;
                break;
            case 2:
                numOfTrajs = 2;
                myTrajectory = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(24)
                        .build();
                myTrajectory2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(26.6)
                        .build();
                waitForStart();
                if(numOfTrajs == 1) {
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();

                } else if(numOfTrajs == 2){
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();

                    drive.followTrajectory(myTrajectory2);
                    PoseStorage.currentPose = drive.getPoseEstimate();

                }else if(numOfTrajs == 3){
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();

                    drive.followTrajectory(myTrajectory2);
                    PoseStorage.currentPose = drive.getPoseEstimate();

                    drive.followTrajectory(myTrajectory3);
                    PoseStorage.currentPose = drive.getPoseEstimate();

                }
                if(isStopRequested()) return;

                break;
            default:
                numOfTrajs = 2;
                myTrajectory = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(24)
                        .build();
                myTrajectory2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(26.6)
                        .build();
                waitForStart();
                if(isStopRequested()) return;
                if(numOfTrajs == 1) {
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                } else if(numOfTrajs == 2){
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.followTrajectory(myTrajectory2);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                }else if(numOfTrajs == 3){
                    drive.followTrajectory(myTrajectory);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.followTrajectory(myTrajectory2);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    drive.followTrajectory(myTrajectory3);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                }
                break;


        }
        PoseStorage.team = team % 2;
    }



    public Pose2d StartingPos() {
        double x, y, a;
        if (team == 0) {
            y = -63.75;
            a = 90;
            if (side == 1) {
                x = 36;
            } else {
                x = -36;

            }
        } else {
            y = 63.75;
            a = -90;
            if (side == 1) {
                x = 36;

            } else {
                x = -36;
            }
        }

        return new Pose2d(x, y, Math.toRadians(a));
    }

}
