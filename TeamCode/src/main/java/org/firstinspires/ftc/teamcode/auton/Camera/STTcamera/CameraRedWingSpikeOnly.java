package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;


@Autonomous(name = "CameraRedWingSpikeOnly")
public class CameraRedWingSpikeOnly extends LinearOpMode {

    EigenOdometry methods = new EigenOdometry(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;
    private Servo servoMoveGripper;
    private Servo servoIntakeL; // dicht is 0
    private Servo servoIntakeR; // dicht is 1
    private Servo servoChopstickL;
    private Servo servoChopstickR;


    double power = .3;

    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        rightFront = map.get(DcMotorEx.class, "right_front");
        leftBack = map.get(DcMotorEx.class, "left_back");
        rightBack = map.get(DcMotorEx.class, "right_back");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");


        servoMoveGripper = hardwareMap.get(Servo.class, "servoMoveGripper");
        servoIntakeL = hardwareMap.get(Servo.class, "servoIntakeL");
        servoIntakeR = hardwareMap.get(Servo.class, "servoIntakeR");
        servoChopstickL = hardwareMap.get(Servo.class, "servoChopstickL");
        servoChopstickR = hardwareMap.get(Servo.class, "servoChopstickR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoChopstickL.setPosition(0.61);
        servoChopstickR.setPosition(0.19);
        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);
        servoMoveGripper.setPosition(0.2);


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
            time = System.currentTimeMillis();
            if (finalPos == 0) {
                methods.driveDean(-55 ,75);
                methods.rotateToHeading(90);
                methods.Stop();
                servoIntakeL.setPosition(1);
                sleep(300);
                methods.driveDean(0,15);
                methods.Stop();
                servoIntakeL.setPosition(0);
                sleep(300);
                methods.Stop();
                terminateOpModeNow();

/*
                servoChopstickL.setPosition(1);
                servoChopstickR.setPosition(0);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(0);
                sleep(50);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(300);
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(400);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1700);
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(1300);
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(240);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                servoIntakeR.setPosition(0);
                sleep(200);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(250);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
*/
            } else if (finalPos == 1) {
                methods.driveDean(-32,90);
                methods.rotateToHeading(90);
                methods.Stop();
                servoIntakeL.setPosition(1);
                sleep(300);
                methods.driveDean(0,15);
                methods.Stop();
                servoIntakeL.setPosition(0);
                sleep(300);
                methods.Stop();
                terminateOpModeNow();

/*
                servoChopstickL.setPosition(1);
                servoChopstickR.setPosition(0);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(0);
                sleep(50);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(300);
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(400);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(2680);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                servoIntakeR.setPosition(0);
                sleep(200);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(250);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
*/
            } else if (finalPos == 2){
                methods.driveDean(-18,70);
                methods.rotateToHeading(90);
                methods.driveDean(0,-20);
                methods.Stop();
                servoIntakeL.setPosition(0.7);
                sleep(300);
                methods.driveDean(0,15);
                methods.Stop();
                servoIntakeL.setPosition(0);
                sleep(300);
                methods.Stop();
                terminateOpModeNow();

/*
                servoChopstickL.setPosition(1);
                servoChopstickR.setPosition(0);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(0);
                sleep(50);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(300);
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(800);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1300);
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(1650);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                servoIntakeR.setPosition(0);
                sleep(200);
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(650);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(250);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

*/
            }
            sleep(30000);
        }
    }
}

