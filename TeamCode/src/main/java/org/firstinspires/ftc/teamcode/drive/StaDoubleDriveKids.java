package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
//import org.firstinspires.ftc.teamcode.robotParts.Limits;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;

@TeleOp(name = "StaDoubleDriveKids",group = "TeleOp")
public class StaDoubleDriveKids extends LinearOpMode {
    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
//            intake.init(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x*.5 + -gamepad2.left_stick_x; // y direction is reversed
            double x = gamepad1.left_stick_y*.5 + gamepad2.left_stick_y;
            double rotate = -gamepad1.right_stick_x*.5 + -gamepad2.right_stick_x;

            boolean servoIntakeOn = gamepad1.a = gamepad2.a;
            boolean servoIntakeOff = gamepad1.b = gamepad2.b;

            boolean servoVliegtuigTrigger = gamepad1.left_bumper = gamepad2.left_bumper;

            if (gamepad2.dpad_down){
                stop();
                sleep(100);
            }

            if(gamepad2.left_stick_button){
                gamepad1.reset();
            }
            if (gamepad2.right_stick_button) {
                gamepad1.reset();
            }
            if (gamepad2.dpad_down) {
                terminateOpModeNow();
            }


            if(servoIntakeOn){
                arm.servoIntake(1);
            } else if (servoIntakeOff) {
                arm.servoIntake(0);
            }

            if(servoVliegtuigTrigger){
                arm.servoVliegtuigHouder(1);
                sleep(200);
                arm.servoVliegtuig(1);
                sleep(800);
                arm.servoVliegtuig(0);
                arm.servoVliegtuigHouder(0);
            }
            drivetrain.drive(x, y, rotate);
            telemetry.update();
        }
    }
}