package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
//import org.firstinspires.ftc.teamcode.robotParts.Limits;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;

@Disabled @TeleOp(name = "staOnlyDriveAndIntake",group = "TeleOp")
public class staOnlyDriveAndIntake extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;

            drivetrain.drive(x, y, rotate);


            boolean intakeOn = gamepad2.left_bumper;
            boolean intakeOff = gamepad2.right_bumper;

            if (intakeOn) {
                arm.intakeL(1);
                arm.intakeR(0);
            } else if (intakeOff) {
                arm.intakeL(0);
                arm.intakeR(1);
            }
        }
    }
}
