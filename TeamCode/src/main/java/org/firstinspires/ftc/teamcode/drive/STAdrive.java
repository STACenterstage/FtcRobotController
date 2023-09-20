package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;
//import org.firstinspires.ftc.teamcode.robotParts.Intake;

public class STAdrive {
    @TeleOp
    public static class STAdrive2 extends LinearOpMode {
        Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
        Arm arm = new Arm();
//        Intake intake = new Intake();

        @Override
        public void runOpMode() throws InterruptedException {
            drivetrain.init(hardwareMap);
            arm.init(hardwareMap);
//            intake.init(hardwareMap);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = gamepad1.left_stick_x; // y direction is reversed
                double x = gamepad1.left_stick_y;
                double rotate = gamepad1.right_stick_x;

                double armPower = gamepad1.right_trigger - gamepad1.left_trigger;

                drivetrain.drive(x, y, rotate);
                arm.move(armPower);
                telemetry.update();
            }
        }
    }
}