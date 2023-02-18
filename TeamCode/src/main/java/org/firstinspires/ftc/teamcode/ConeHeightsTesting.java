package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class ConeHeightsTesting extends DriveCodeCommonNotBryce {
    boolean open = false;
    boolean wasOpened = false;
//    public static final double[] coneHeights = {0,0,0.1075,0.1774,0.2401,0.3458};
//    public static final double[] coneHeightsClear = {0,0.05,0.3512,0.4659,0.4838,0.5197};
//    int coneHeightsNum = 5;
//    boolean clearing = true;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            if(gamepad2.a) {
                drive.slift.setPosition(drive.slift.getPosition()+0.001);
            } else if(gamepad2.b){
                drive.slift.setPosition(drive.slift.getPosition()-0.001);
            }

            telemetry.addData("slift",drive.slift.getPosition());
//            telemetry.addData("coneNum",coneHeightsNum);
//            telemetry.addData("clearing",clearing);
            telemetry.update();

            if(button_x2_was_pressed && !open && !wasOpened) {
                SClaw(true);
                open = true;
                wasOpened = true;
            } else if(button_x2_was_pressed && open && !wasOpened) {
                SClaw(false);
                open = false;
                wasOpened = true;
            } else if(!button_x2_was_pressed && wasOpened) {
                wasOpened = false;
            }

            if (gamepad2.x && !button_x2_was_pressed) {
                button_x2_was_pressed = true;
            } else if (!gamepad2.x && button_x2_was_pressed) {
                button_x2_was_pressed = false;
            }
//            if (gamepad2.y && !button_y2_was_pressed) {
//                if(clearing) {
//                    clearing = false;
//                } else {
//                    clearing = true;
//                }
//                button_y2_was_pressed = true;
//            } else if (!gamepad2.y && button_y2_was_pressed) {
//                button_y2_was_pressed = false;
//            }
//            if (gamepad2.a && !button_a2_was_pressed && coneHeightsNum < 5) {
//                coneHeightsNum++;
//                button_a2_was_pressed = true;
//            } else if (!gamepad2.a && button_a2_was_pressed) {
//                button_a2_was_pressed = false;
//            }
//            if (gamepad2.b && !button_b2_was_pressed && coneHeightsNum > 1) {
//                coneHeightsNum--;
//                button_b2_was_pressed = true;
//            } else if (!gamepad2.b && button_b2_was_pressed) {
//                button_b2_was_pressed = false;
//            }
        }
    }
}