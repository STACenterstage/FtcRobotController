package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
//import org.firstinspires.ftc.teamcode.robotParts.Limits;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;

@TeleOp(name = "STAdrive",group = "TeleOp")
public class STAdrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();
    boolean climbMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
//            intake.init(hardwareMap);


        telemetry.addData("ArmPos", arm.ArmPos());
        telemetry.addData("MoveGripperPos", arm.MoveGripperPos());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x; // y direction is reversed
            double x = gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;

            boolean holdChange = gamepad2.dpad_down;
            boolean holdOff = gamepad2.dpad_up;

            boolean servoIntakeOn = gamepad1.a;
            boolean servoIntakeOff = gamepad1.b;

            boolean servoVliegtuigTrigger = gamepad1.left_bumper;


            //boolean spoelNegativePower = gamepad2.left_bumper;
            //boolean spoelPositivePower = gamepad2.right_bumper;

            double armPower = (gamepad2.right_trigger - gamepad2.left_trigger);

            boolean chopstickOn = gamepad2.y;
            boolean chopstickLOff = gamepad2.right_bumper;
            boolean chopstickROff = gamepad2.left_bumper;


            if (chopstickOn) {
                arm.chopstickL(0.45);
                arm.chopstickR(0.32);
            } else {
                if (chopstickLOff) {
                    arm.chopstickL(0.61);
                }
                if (chopstickROff) {
                    arm.chopstickR(0.19);
                }
            }

            boolean intakeOn = gamepad1.a;
            boolean intakeOff = gamepad1.b;

            if (intakeOn) {
                arm.intakeL(1);
                arm.intakeR(0);
            } else if (intakeOff) {
                arm.intakeL(0);
                arm.intakeR(1);
            }


            /*
            if(runtime.milliseconds() > 2500) {
                if (spoelPositivePower) {
                    arm.spoelPositivePower(1);
                } else if (spoelNegativePower) {
                    arm.spoelNegativePower(1);
                } else {
                    arm.spoelNegativePower(0);
                    arm.spoelPositivePower(0);
                }
            }
            */

            if (servoIntakeOn) {
                arm.servoIntake(1);
            } else if (servoIntakeOff) {
                arm.servoIntake(0);
            }

            if (servoVliegtuigTrigger) {
                arm.servoVliegtuigHouder(1);
                sleep(80);
                arm.servoVliegtuig(1);
                sleep(920);
                arm.servoVliegtuig(0);
                arm.servoVliegtuigHouder(0);
            }

//            if (arm.ArmPos() < 2550) {
//                arm.moveGripper(0);
//            } else if (arm.ArmPos() > 2600) {
//                arm.moveGripper((0.000275 * arm.ArmPos()*-1+1) + 0.7);
//            }
//                arm.moveGripper(-0.000233 * arm.ArmPos() + 0.3433);

//            if (arm.ArmPos() < 400) {
//                arm.moveGripper(0.00 + gamepad2.left_stick_x*0.05);
////              was 0.1
//            } else if (arm.ArmPos() > 2400) {
//                arm.moveGripper((0.000275 * arm.ArmPos()*-1+1) + 0.88 + gamepad2.left_stick_x*0.1);
//            } else {
//                arm.moveGripper(.0005 * (arm.ArmPos() - 400));
//            }
            if (gamepad2.dpad_down){
                climbMode = true;
            }else if (gamepad2.dpad_up){
                climbMode = false;
            }

            if (climbMode){
                arm.moveGripper(0.12);
            } else if (arm.ArmPos() < 400) {
                arm.moveGripper(0.245 + gamepad2.left_stick_x * 0.02);
//              was 0.1
            } else if (arm.ArmPos() > 2300) {
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.18 + gamepad2.left_stick_x * 0.06);
            } else {
                arm.moveGripper(.0003 * (arm.ArmPos() - 400) + 0.26);
            }

//
//            if (intakeBtn)
//                if (til) {
//                btnMode = true;+
//                armHeight = -300;
//            } else if (pak) {
//                btnMode = true;
//                armHeight = -500;
//            }
//
//            if (Math.abs(armPower) > 0.1) {
//                btnMode = false;
//            }

            telemetry.addData("climbMode", climbMode);
            telemetry.addData("ArmPos", arm.ArmPos());
            telemetry.addData("MoveGripperPos", arm.MoveGripperPos());
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();

        }
    }
}
