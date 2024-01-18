package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;

import org.firstinspires.ftc.teamcode.drive.STAdrive;

@Autonomous(name="AutonomousRedBackstage", group="LinearOpmode")
public class AutonomousRedBackstage extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;
    private Servo servoGripper;
    private Servo servoMoveGripper;
    private Servo servoIntakeL; // dicht is 0
    private Servo servoIntakeR; // dicht is 1
    private Servo servoChopstickL;
    private Servo servoChopstickR;


    double power = .25;

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


        servoChopstickL.setPosition(0.47);
        servoChopstickR.setPosition(0.3);
        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);
        servoMoveGripper.setPosition(.82);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        

        /*leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }



    @Override
    public void runOpMode() {
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        init(hardwareMap);
        arm.init(hardwareMap);
        drivetrain.init(hardwareMap);

        waitForStart();
        runtime.reset();


        telemetry.addData("ArmPos",arm.ArmPos());
        telemetry.update();



        /*
        while (opModeIsActive()) {
            telemetry.addData("ArmPos", arm.ArmPos());
            telemetry.update();
        }
        */

        //pak pixel en wacht
        servoChopstickL.setPosition(0.47);
        servoChopstickR.setPosition(0.3);
        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);
        servoMoveGripper.setPosition(.82);
        sleep(100);


        telemetry.addData("ArmPos",arm.ArmPos());
        telemetry.update();


        //Vanuit startpositie naar links
        leftFront.setPower(-1.2*power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
        sleep(3100);

        //Intake open voor loslaten paarse pixel
        servoIntakeL.setPosition(1);
        servoIntakeR.setPosition(0);

        //stoppen
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(400);

        //stukje naar voren voor pixel
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(600);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        sleep(2000);

        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);

        //Naar achteren rijden
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        sleep(3500);

        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(80);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(80);

        //arm omhoog
        while (arm.ArmPos() < 2200){
            arm1.setPower(.7);
        }
        arm1.setPower(0);


        sleep(100);
        servoMoveGripper.setPosition(0.000275*arm.ArmPos()-0.75);
        telemetry.addData("ArmPos",arm.ArmPos());
        telemetry.update();
        sleep(400);

        //Alles uit
        arm1.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(1000);

        //laat gele pixel los
        servoChopstickL.setPosition(1);
        servoChopstickR.setPosition(0);
        sleep(800);

        //alles uit
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(3000);

        //kléin stukje naar voren
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(1000);

        //Klein stukje naar links
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
        sleep(2800);

        //Arm in
        while (arm.ArmPos() > 500){
            arm1.setPower(-.7);
        }
        arm1.setPower(0);

        sleep(100);
        servoMoveGripper.setPosition(.82);
        telemetry.addData("ArmPos",arm.ArmPos());
        telemetry.update();
        sleep(400);

        //Naar achteren rijden
        arm1.setPower(0);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        sleep(3000);

        //alles uit
        leftFront.setPower(-0);
        rightFront.setPower(-0);
        leftBack.setPower(-0);
        rightBack.setPower(-0);
        sleep(400);

        arm1.setPower(0);
        servoMoveGripper.setPosition(1);


    }
}


          /*

            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*    leftFront.setPower(20000);
            rightFront.setPower(20000);
            leftBack.setPower(20000);
            rightBack.setPower(20000);
            sleep( 500);


            leftFront.setPower(300);
            rightFront.setPower(0);
            leftBack.setPower(3);
            rightBack.setPower(0);

            leftBack.setPower(0);
            leftFront.setPower(0);

            leftFront.setPower(7);
            rightFront.setPower(7);
            leftBack.setPower(7);
            rightBack.setPower(7);

            arm1.setPower(power);

            arm1.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);*/

