package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Auto extends LinearOpMode {
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
    int xShift = 0;
    int headingMod = 0;

    public int Team() {
        return team % 2;
    }

    public int Side() {
        return side % 2;
    }

    public int Mode() {
        return Mode % 3;
    }
    OpenCvWebcam webcam;
    SleeveOrientationPipeline pipeline;
    @Override
    public void runOpMode() {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field

        drive.setPoseEstimate(StartingPos());

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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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
                Mode++;
                button_x_was_pressed = true;
            } else if (!gamepad1.x && button_x_was_pressed) {
                button_x_was_pressed = false;
            }
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
                            }
                            break;
                    }
                    break;
            }
            telemetry.addData("Robot Position: ", drive.getPoseEstimate());
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
        }

        if (team % 2 == 1) {
            yMod = 1;
        } else {
            yMod = -1;
        }
        if ((team % 2 == 0 && side % 2 == 0) || (team % 2 == 1 && side % 2 == 1)) {
            xReflect = -1;
            xShift = 0;
            headingMod = 0;
        } else if ((team % 2 == 0 && side % 2 == 1) || (team % 2 == 1 && side % 2 == 0)) {
            xReflect = 1;
            xShift = 72;
            headingMod = 180;
        }

waitForStart();
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

        Pose2d movement0 = new Pose2d(36 * xReflect, 60 * yMod, Math.toRadians(-90*yMod));
        Pose2d movement1 = new Pose2d(12 * xReflect, 60 * yMod, Math.toRadians(-90*yMod));
        Pose2d movement2 = new Pose2d(12 * xReflect, 12 * yMod, Math.toRadians(-90*yMod));
        Pose2d scorePos = new Pose2d(24 * xReflect, 12 * yMod, Math.toRadians(180 + headingMod));    //Score Position
        Pose2d intakeStackPos = new Pose2d(62 * xReflect, 12 * yMod, Math.toRadians(180 + headingMod));    //Intake cone stack Position
        Pose2d park1 = new Pose2d((60*xReflect) + xShift, 12 * yMod, Math.toRadians(-90*yMod));
        Pose2d park2 = new Pose2d((36*xReflect) + xShift, 12 * yMod, Math.toRadians(-90*yMod));
        Pose2d park3 = new Pose2d((12*xReflect) + xShift, 12 * yMod, Math.toRadians(-90*yMod));
        Trajectory Movement0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(movement0)
                .build();
        Trajectory Movement1 = drive.trajectoryBuilder(movement0)
                .lineToLinearHeading(movement1)
                .build();
        Trajectory Movement2 = drive.trajectoryBuilder(Movement1.end())
                .lineToLinearHeading(movement2)
                .build();
        Trajectory ScorePos1 = drive.trajectoryBuilder(Movement1.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory IntakeStackPos1 = drive.trajectoryBuilder(ScorePos1.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory ScorePos2 = drive.trajectoryBuilder(IntakeStackPos1.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory IntakeStackPos2 = drive.trajectoryBuilder(ScorePos2.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory ScorePos3 = drive.trajectoryBuilder(IntakeStackPos2.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory IntakeStackPos3 = drive.trajectoryBuilder(ScorePos3.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory ScorePos4 = drive.trajectoryBuilder(IntakeStackPos3.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory IntakeStackPos4 = drive.trajectoryBuilder(ScorePos4.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory ScorePos5 = drive.trajectoryBuilder(IntakeStackPos4.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory IntakeStackPos5 = drive.trajectoryBuilder(ScorePos5.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory ScorePos6 = drive.trajectoryBuilder(IntakeStackPos5.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory Park1 = drive.trajectoryBuilder(Movement2.end())
                .lineToLinearHeading(park1)
                .build();
        Trajectory Park2 = drive.trajectoryBuilder(Movement2.end())
                .lineToLinearHeading(park2)
                .build();
//        Trajectory Park3 = drive.trajectoryBuilder(Movement2.end())
//                .lineToLinearHeading(park3)
//                .build();
//        Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .forward(2)
//                .build();
//        Trajectory Movement2 = drive.trajectoryBuilder(Movement1.end())
//                .strafeRight(24*6*xReflect)
//                .build();
//        Trajectory Movement3 = drive.trajectoryBuilder(Movement1.end())
//                .back(10*5)
//                .build();
//        Trajectory Movement4 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .forward(70)
//                .build();
//        Trajectory Park1 = drive.trajectoryBuilder(Movement4.end())
//                .strafeLeft(155)
//                .build();
////        Trajectory Park2 = drive.trajectoryBuilder(Movement4.end())
////                .strafeLeft(24*6*xReflect)
////                .build();
//        Trajectory Park3 = drive.trajectoryBuilder(Movement4.end())
//                .strafeRight(155)
//                .build();
        if (Mode % 3 == 0) {//doing nothing

        } else if (Mode % 3 == 1) {//cycling
            drive.followTrajectory(Movement0);
            drive.followTrajectory(Movement1);
            drive.followTrajectory(Movement2);
            drive.followTrajectory(ScorePos1);
            //Score Cone
            drive.followTrajectory(IntakeStackPos1);
            //Intake Cone
            drive.followTrajectory(ScorePos2);
            //Score Cone
            drive.followTrajectory(IntakeStackPos2);
            //Intake Cone
            drive.followTrajectory(ScorePos3);
            //Score Cone
            drive.followTrajectory(IntakeStackPos3);
            //Intake Cone
            drive.followTrajectory(ScorePos4);
            //Score Cone
            drive.followTrajectory(IntakeStackPos4);
            //Intake Cone
            drive.followTrajectory(ScorePos5);
            //Score Cone

            drive.followTrajectory(IntakeStackPos5);
            //Intake Cone
            drive.followTrajectory(ScorePos6);
            //Score Cone

            switch (autoParkPosition) {
                case 1:
                    drive.followTrajectory(Park2);
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (side % 2 == 0) {
//                        drive.followTrajectory(Park3);
                    } else {
                        drive.followTrajectory(Park1);
                    }
                    if (isStopRequested()) return;
                    break;
                default:
                    if (side % 2 == 0) {
                        drive.followTrajectory(Park1);
                    } else {
//                        drive.followTrajectory(Park3);
                    }
                    if (isStopRequested()) return;
                    break;
            }
        } else if (Mode % 3 == 2) {//Just Parking
            drive.followTrajectory(Movement0);
            drive.followTrajectory(Movement1);
            drive.followTrajectory(Movement2);

            switch (autoParkPosition) {
                case 1:
                    drive.followTrajectory(Park2);
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (side % 2 == 0) {
//                        drive.followTrajectory(Park3);
                    } else {
                        drive.followTrajectory(Park1);
                    }
                    if (isStopRequested()) return;
                    break;
                default:
                    if (side % 2 == 0) {
                        drive.followTrajectory(Park1);
                    } else {
//                        drive.followTrajectory(Park3);
                    }
                    if (isStopRequested()) return;
                    break;
            }
        }

        PoseStorage.team = team % 2;
    }


    public Pose2d StartingPos() {
        double x, y, a;
        if (team == 0) {
            y = -63.75;
            a = 90;
            if (side == 1) {
                x = 40;
            } else {
                x = -40;

            }
        } else {
            y = 63.75;
            a = -90;
            if (side == 1) {
                x = -40;

            } else {
                x = 40;
            }
        }

        return new Pose2d(x, y, Math.toRadians(a));
    }

    public static class SleeveOrientationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the Sleeve Orientation
         */

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140,75);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(140,140);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(140,200);
        static final int REGION_WIDTH = 125;
        static final int REGION_HEIGHT = 60;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile int position = 0;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = 0; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = 1; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = 2; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public int getAnalysis()
        {
            return position;
        }
    }
}
