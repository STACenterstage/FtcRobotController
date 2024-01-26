package org.firstinspires.ftc.teamcode.auton.odometry;

import static org.firstinspires.ftc.teamcode.auton.odometry.PixelManipulation.ArmHeight.FIRSTLINE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.odometry.newAutonMethods;
import org.firstinspires.ftc.teamcode.auton.odometry.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.auton.odometry.PixelManipulation;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    PixelManipulation slides = new PixelManipulation(this);

    @Override
    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap, telemetry);
        methods.calibrateEncoders();
        methods.resetYaw();
        slides.claw.setPosition(PixelManipulation.ClawPositions.AUTONSTART.getPosition());
        slides.updateElbow(PixelManipulation.ElbowPositions.AUTONSTART);
        slides.wrist.setPosition(0.68);
        waitForStart();
        if (opModeIsActive()){
            slides.dropYellowPixel();
        }
    }
}