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


@Autonomous(name = "CBlueWing")
public class CBlueWing extends LinearOpMode {

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
    double time;


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

        servoChopstickL.setPosition(0.45); // 0.45 = on, 0.61 = off
        servoChopstickR.setPosition(0.19); // 0.32 = on, 0.19 = off
        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);
        servoMoveGripper.setPosition(0.2);

        telemetry.addLine("Paarse Pixel moet LINKS!");
        telemetry.update();
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

                methods.driveDean(0,45);
                methods.rotateToHeading(-135);
                methods.Stop();
                methods.driveDean(0,-34);
                methods.Stop();
                servoIntakeL.setPosition(0.7);
                sleep(300);
                methods.driveDean(0,50);
                methods.Stop();
                servoIntakeL.setPosition(0);
                sleep(300);
                methods.driveDean(-40,-85);
                methods.rotateToHeading(90);
                methods.driveDean(0,63); //todo: Afstand tot backboard.
                while ((System.currentTimeMillis() < time + 16000) && !isStopRequested()){
                    methods.Stop();
                    sleep(100);
                }
                methods.driveDean(0,105);
                methods.driveDean(-80,40);
                methods.Stop();
                sleep(300);
                while (arm.ArmPos() < 3200 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickL.setPosition(0.61);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 300 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickL.setPosition(0.61);
                sleep(300);
                methods.driveDean(0,15);
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
                sleep(1850);            //todo: Dit aanpassen als te ver
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(1220);
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(600);
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

                methods.driveDean(-8,112);
                methods.Stop();
                servoIntakeL.setPosition(0.7);
                sleep(300);
                methods.driveDean(0,20);
                methods.rotateToHeading(90);
                methods.Stop();
                servoIntakeL.setPosition(0);
                sleep(300);
                methods.driveDean(0,65); //todo: Afstand tot backboard.
                while ((System.currentTimeMillis() < time + 16000) && !isStopRequested()){
                    methods.Stop();
                    sleep(100);
                }
                methods.driveDean(0,105);
                methods.driveDean(-58,40);
                methods.Stop();
                sleep(300);
                while (arm.ArmPos() < 3200 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickL.setPosition(0.61);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 300 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickL.setPosition(0.61);
                sleep(300);
                methods.driveDean(0,15);
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
                sleep(2850);            //todo: Dit aanpassen als te ver

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
                methods.driveDean(10,95);
                servoIntakeL.setPosition(0.7);
                methods.driveDean(0,40);
                methods.rotateToHeading(90);
                methods.Stop();
                servoIntakeL.setPosition(0);
                sleep(300);
                methods.driveDean(0,93); //todo: Afstand tot backboard.
                while ((System.currentTimeMillis() < time + 16000) && !isStopRequested()){
                    methods.Stop();
                    sleep(100);
                }
                methods.driveDean(0,105);
                methods.driveDean(-42,40);
                methods.Stop();
                sleep(300);
                while (arm.ArmPos() < 3200 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickL.setPosition(0.61);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 300 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickL.setPosition(0.61);
                sleep(300);
                methods.driveDean(0,15);
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
                sleep(2200);            //todo: Dit aanpassen als te ver

                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(1250);

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
            }
            sleep(30000);
        }
    }
}

