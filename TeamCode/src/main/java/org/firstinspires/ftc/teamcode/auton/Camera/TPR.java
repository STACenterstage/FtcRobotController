package org.firstinspires.ftc.teamcode.auton.Camera;


import org.firstinspires.ftc.teamcode.auton.Camera.OpenCVRandomization;
import org.firstinspires.ftc.teamcode.auton.Camera.newAutonMethods;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TPR")
public class TPR extends LinearOpMode {
    OpenCVRandomization camera = new OpenCVRandomization(this);
    newAutonMethods methods = new newAutonMethods(this);

    public void runOpMode() {
        methods.init(hardwareMap);
        methods.calibrateEncoders();

        methods.resetIMU(hardwareMap);
        camera.findScoringPosition();
        // kijk of je assen kloppen de XYZ van imu, anders doet ie niks
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("localPos", camera.pos);
            methods.driveX(30 - (methods.robotLength_cm), 0.3, telemetry);
            if (camera.pos == 0) {
                methods.driveY(-90 + (methods.robotLength_cm), 0.3, telemetry);
                methods.rotateToHeading(-90, 0.2, telemetry);
            } else if (camera.pos == 1) {
                methods.driveY(-120, 0.3, telemetry);
            } else if (camera.pos == 2){
                methods.driveY(-90 + (methods.robotLength_cm), 0.3, telemetry);
                methods.rotateToHeading(90, 0.2, telemetry);
                methods.driveY(-30 + (methods.robotLength_cm), 0.3, telemetry);
            }
            if (camera.pos == 1 || camera.pos == 2) {}
            //rest of shit
            sleep(30000);
        }
    }
}
