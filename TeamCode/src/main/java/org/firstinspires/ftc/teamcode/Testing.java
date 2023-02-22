package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class Testing extends DriveCodeCommonNotBryce {
    String mode = "teleOp";
    int tPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            RawDriving();
            Toggles();

            if(mode == "teleop") {
                ShootOut();
            } else {
                if(gamepad1.left_bumper) {
                    tPos = 0;
                } else if(gamepad1.right_bumper) {
                    tPos = 1900;
                }
                drive.shooter.setTargetPosition(tPos);
                drive.shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.shooter.setPower(1);
            }

            if(button_a_was_pressed) {
                if(mode == "teleOp") {
                    mode = "auto";
                } else {
                    mode = "teleOp";
                }
            }
            telemetry.addData("shooterPos",drive.shooter.getCurrentPosition());
            telemetry.addData("Mode:",mode);
            telemetry.update();


        }
    }
}