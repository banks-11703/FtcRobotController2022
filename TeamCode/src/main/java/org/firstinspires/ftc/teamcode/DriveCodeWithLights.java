package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class DriveCodeWithLights extends DriveCodeCommon {
    @Override
    public void runOpMode() {
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            Lift();
            Claw();
            TurnTable();
            Lights();
            Telemetry();
        }
    }
}

