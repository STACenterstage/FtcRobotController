package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
//import org.firstinspires.ftc.teamcode.robotParts.Limits;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;

@TeleOp(name = "STAdriveKids",group = "TeleOp")
public class STAdriveKids extends LinearOpMode {
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
            double y = -gamepad1.left_stick_x*.5; // y direction is reversed
            double x = gamepad1.left_stick_y*.5;
            double rotate = -gamepad1.right_stick_x*.5;
            boolean grab = gamepad2.a;
            boolean release = gamepad2.b;
            double moveGripper = gamepad2.left_stick_x *-.8 +1;

            boolean servoIntakeOn = gamepad1.a;
            boolean servoIntakeOff = gamepad1.b;

            boolean servoVliegtuigTrigger = gamepad1.left_bumper;

            double armPower = gamepad2.right_trigger*.8 - gamepad2.left_trigger*.8;

            if(grab){
                arm.gripper(1);
            } else if (release) {
                arm.gripper(0);
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

            arm.moveGripper(moveGripper);
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();
        }
    }
}