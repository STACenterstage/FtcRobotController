package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
//import org.firstinspires.ftc.teamcode.robotParts.Limits;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;

@TeleOp(name = "STAdrive",group = "TeleOp")
public class STAdrive extends LinearOpMode {
    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();
    boolean hold = false;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
//            intake.init(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x; // y direction is reversed
            double x = gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;
            boolean grab = gamepad2.a;
            boolean release = gamepad2.b;
            double moveGripper = gamepad2.left_stick_x *-1 +1;
            boolean holdChange = gamepad2.dpad_down;
            boolean holdOff = gamepad2.dpad_up;

            boolean servoIntakeOn = gamepad1.a;
            boolean servoIntakeOff = gamepad1.b;

            boolean servoVliegtuigTrigger = gamepad1.left_bumper;


            double armPower = gamepad2.right_trigger - gamepad2.left_trigger;

            boolean GripperHoldOn = gamepad2.left_bumper;
            boolean GripperHoldOff = gamepad2.right_bumper;


            if(grab){
                arm.gripper(.45);
            } else if (release) {
                arm.gripper(.2);
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

            if(holdChange){
                hold = true;
            }
            else if(holdOff){
                hold = false;
            }

            if(GripperHoldOn){
                arm.servoGripperHold(1);
            }
            else if(GripperHoldOff){
                arm.servoGripperHold(0);
            }

            if(hold){
                arm.hold();
            }

            arm.moveGripper(moveGripper);
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();




        }
    }
}