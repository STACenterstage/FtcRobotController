package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain_weirdo;
//import org.firstinspires.ftc.teamcode.robotParts.Drivetrain_weirdo_FieldCentric;

@TeleOp(name = "WeirdoDrive",group = "TeleOp")
public class WeirdoDrive extends LinearOpMode {
    Drivetrain_weirdo.drivetrain drivetrain_weirdo = new Drivetrain_weirdo.drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain_weirdo.init(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;
            double rotate = 0.7*gamepad1.right_stick_x;
            drivetrain_weirdo.drive(x, y, rotate);
//            Drivetrain_weirdo_FieldCentric.FieldCentric();
        }
    }
}
