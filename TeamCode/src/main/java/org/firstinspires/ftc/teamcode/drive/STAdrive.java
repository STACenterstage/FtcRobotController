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
    boolean btnMode = false;
    int armHeight;

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

            boolean intakeBtn = gamepad2.x;
            boolean til = gamepad2.dpad_left;
            boolean pak = gamepad2.dpad_right;

            boolean chopstickOn = gamepad2.y;
            boolean chopstickLOff = gamepad2.left_bumper;
            boolean chopstickROff = gamepad2.right_bumper;


            if (chopstickOn) {
                arm.chopstickL(0.47);
                arm.chopstickR(0.3);
            } else {
                if (chopstickLOff) {
                    arm.chopstickL(1);
                }
                if (chopstickROff) {
                    arm.chopstickR(0);
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

            if (arm.ArmPos() < 2550) {
                arm.moveGripper(.78);
            } else if (arm.ArmPos() > 2600) {
                arm.moveGripper(0.000275 * arm.ArmPos() - 0.745);
            }

            /*if (intakeBtn)
                if (til) {
                btnMode = true;+
                armHeight = -300;
            } else if (pak) {
                btnMode = true;
                armHeight = -500;
            }

            if (Math.abs(armPower) > 0.1) {
                btnMode = false;
            }
*/
            telemetry.addData("ArmPos", arm.ArmPos());
            telemetry.addData("MoveGripperPos", arm.MoveGripperPos());
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();

        }
    }
}
