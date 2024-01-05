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


            double moveGripper = gamepad2.left_stick_x *-1 +1;
            boolean holdChange = gamepad2.dpad_down;
            boolean holdOff = gamepad2.dpad_up;

            boolean servoIntakeOn = gamepad1.a;
            boolean servoIntakeOff = gamepad1.b;

            boolean servoVliegtuigTrigger = gamepad1.left_bumper;

            //boolean spoelNegativePower = gamepad2.left_bumper;
            //boolean spoelPositivePower = gamepad2.right_bumper;

            double armPower = (gamepad2.right_trigger - gamepad2.left_trigger);

            boolean chopstickOn = gamepad2.y;
            boolean chopstickLOff = gamepad2.left_bumper;
            boolean chopstickROff = gamepad2.right_bumper;



            if(chopstickOn){
                arm.ChopstickL(0.47);
                arm.ChopstickR(0.3);
            }
            else {
                if (chopstickLOff) {
                    arm.ChopstickL(1);
                }
                if (chopstickROff) {
                    arm.ChopstickR(0);
                }
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

            if(hold){
                arm.hold();
            }

            telemetry.addData("ArmPos",arm.ArmPos());
            arm.moveGripper(moveGripper);
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();

        }
    }
}