package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriveCode extends DriveCodeCommon {
    @Override
    public void runOpMode() {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            if (gamepad2.a||gamepad2.b){
                Lift();
            }
            Claw();
            TurnTable();
            ShootOut();
            Telemetry();
//            ColorSensor();
        }
    }
}