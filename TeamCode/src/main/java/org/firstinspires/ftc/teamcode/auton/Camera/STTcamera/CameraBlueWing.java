package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;


@Autonomous(name = "CameraBlueWing")
public class CameraBlueWing extends LinearOpMode {

    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;

    double power = .3;

    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        rightFront = map.get(DcMotorEx.class, "right_front");
        leftBack = map.get(DcMotorEx.class, "left_back");
        rightBack = map.get(DcMotorEx.class, "right_back");

        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    public void runOpMode() {
        methods.init(hardwareMap);

        methods.calibrateEncoders();
        camera.findScoringPosition();

        init(hardwareMap);
        arm.init(hardwareMap);
        drivetrain.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            int finalPos = camera.pos;
            if (finalPos == 0) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(1500);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            } else if (finalPos == 1) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1500);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            } else if (finalPos == 2){
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(1500);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            }
            sleep(30000);

            telemetry.addData("DOnE","DONE");
            telemetry.update();
//            int finalPos = camera.pos;
//            telemetry.addData("localPos", camera.pos);
//            if (finalPos == 0) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(-90);
//                methods.driveY(-40 + 0.5 * methods.robotLength_cm);
//                methods.driveY(40 - 0.5 * methods.robotLength_cm);
//                methods.driveX(-60 + 0.5 * methods.robotWidth_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-80 + 0.5 * methods.robotLength_cm);
//            } else if (finalPos == 1) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-112 + methods.robotLength_cm);
//                methods.driveY(30);
//                methods.driveX(-60);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-30);
//            } else if (finalPos == 2){
//                methods.driveX(-0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.driveY(30);
//                methods.driveX(-30);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-25);
//            }
            sleep(30000);
        }
    }
}

