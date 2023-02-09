package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NoAutoHomeDriveCode extends DriveCodeCommonForThoseWhoAreNotBryce {
    @Override
    public void runOpMode() {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            Lift();
            Claw();
            TurnTable();
            ShootOut();
            Telemetry();
            ColorSensor();
        }
    }
}