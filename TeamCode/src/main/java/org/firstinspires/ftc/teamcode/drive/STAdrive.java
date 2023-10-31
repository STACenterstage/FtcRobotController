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

            boolean servoIntakeOn = gamepad1.a;
            boolean servoIntakeOff = gamepad1.b;

            double armPower = gamepad2.right_trigger - gamepad2.left_trigger;

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

            arm.moveGripper(moveGripper);
            drivetrain.drive(x, y, rotate);
            arm.move(armPower);
            telemetry.update();




        }
    }
}