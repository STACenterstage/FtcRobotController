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

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x*.5 - gamepad2.left_stick_x; // y direction is reversed
            double x = gamepad1.left_stick_y*.5 + gamepad2.left_stick_y;
            double rotate = -gamepad1.right_stick_x*.5 - gamepad2.right_stick_x;

            double armPower = (gamepad2.right_trigger - gamepad2.left_trigger);

            boolean servoVliegtuigTrigger = gamepad1.left_bumper;

            if(gamepad2.left_stick_button){
                gamepad1.reset();
            }
            if (gamepad2.right_stick_button) {
                gamepad1.reset();
            }
            if (gamepad2.dpad_down) {
                terminateOpModeNow();
            }

            boolean chopstickOn = gamepad2.y;
            boolean chopstickLOff = gamepad2.left_bumper;
            boolean chopstickROff = gamepad2.right_bumper;


            if (chopstickOn) {
                arm.chopstickL(0.47);
                arm.chopstickR(0.3);
            } else {
                if (chopstickLOff) {
                    arm.chopstickL(0.61);
                }
                if (chopstickROff) {
                    arm.chopstickR(0.19);
                }
            }

            boolean intakeOn = gamepad1.a || gamepad2.a;
            boolean intakeOff = gamepad1.b || gamepad2.b;

            if (intakeOn) {
                arm.intakeL(1);
                arm.intakeR(0);
            } else if (intakeOff) {
                arm.intakeL(0);
                arm.intakeR(1);
            }


            if (arm.ArmPos() < 400) {
                arm.moveGripper(0.245);
//              was 0.1
            } else if (arm.ArmPos() > 2300) {
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.18);
            } else {
                arm.moveGripper(.0003 * (arm.ArmPos() - 400) + 0.26);
            }


//                arm.moveGripper(-0.000233 * arm.ArmPos() + 0.3433);

            if(servoVliegtuigTrigger){
                arm.servoVliegtuigHouder(1);
                sleep(200);
                arm.servoVliegtuig(1);
                sleep(800);
                arm.servoVliegtuig(0);
                arm.servoVliegtuigHouder(0);
            }
            telemetry.addData("ArmPos", arm.ArmPos());
            telemetry.addData("MoveGripperPos", arm.MoveGripperPos());
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();
        }
    }
}