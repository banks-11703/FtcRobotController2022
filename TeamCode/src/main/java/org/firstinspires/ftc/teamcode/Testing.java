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
    //p=10,i=0.049988,d=0,f=0
    public static double p = 10;//10
    public static double I = 0.049988;//0.049988
    public static double d = 0;//0
    public static double f = 0;//0
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
                    tPos = -1900;
                }
                drive.shooter.setVelocityPIDFCoefficients(p,I,d,f);
                drive.shooter.setPositionPIDFCoefficients(p);

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
            telemetry.addData("PID for shooter",drive.shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.update();


        }
    }
}