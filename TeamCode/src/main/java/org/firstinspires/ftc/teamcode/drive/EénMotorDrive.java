package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;
import org.firstinspires.ftc.teamcode.robotParts.EénMotor;


@TeleOp(name = "EénMotorDrive",group = "TeleOp")
public class EénMotorDrive extends LinearOpMode {
    EénMotor.drivetrain drivetrain = new EénMotor.drivetrain();
    Arm arm = new Arm();
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
//            intake.init(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;
        }
    }}