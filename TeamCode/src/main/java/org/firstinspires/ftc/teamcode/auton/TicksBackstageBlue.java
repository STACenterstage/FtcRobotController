package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled //(name="TicksBackstageBlue", group="LinearOpmode")
public class TicksBackstageBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;
    private Servo servoGripper;
    private Servo servoMoveGripper;
    private Servo servoIntake;
    double power = 1;

    private double servoGripperMin = 0.0;  // Minimum position for servo1 (in degrees)
    private double servoGripperMax = 180.0;  // Maximum position for servo1 (in degrees)
    private double servoMoveGripperMin = 0.0;  // Minimum position for servo2 (in degrees)
    private double servoMoveGripperMax = 180.0;  // Maximum position for servo2 (in degrees)

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        rightFront = map.get(DcMotorEx.class, "right_front");
        leftBack = map.get(DcMotorEx.class, "left_back");
        rightBack = map.get(DcMotorEx.class, "right_back");

        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");

        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoMoveGripper = hardwareMap.get(Servo.class, "servoMoveGripper");
        servoIntake = hardwareMap.get(Servo.class, "Intake");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }


    public void drive(int leftBackTicks, int rightBackTicks, int leftFrontTicks, int rightFrontTicks) {
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

        leftBack.setTargetPosition(leftBackTicks);
        rightBack.setTargetPosition(rightBackTicks);
        leftFront.setTargetPosition(leftFrontTicks);
        rightFront.setTargetPosition(rightFrontTicks);

        //DriveFront
        if (leftBackTicks < 0 && leftFrontTicks < 0 && rightBackTicks < 0 && rightFrontTicks < 0) {
            leftBack.setPower(.28);
            rightBack.setPower(.67);
            leftFront.setPower(.62);
            rightFront.setPower(.67);
        }
        //DriveBack
        if (leftBackTicks > 0 && leftFrontTicks > 0 && rightBackTicks > 0 && rightFrontTicks > 0) {
            leftBack.setPower(.28);
            rightBack.setPower(.86);
            leftFront.setPower(.62);
            rightFront.setPower(.86);
        }
        //DriveLeft
        if (leftBackTicks < 0 && leftFrontTicks > 0 && rightBackTicks > 0 && rightFrontTicks < 0) {
            leftBack.setPower(.28);
            rightBack.setPower(.67);
            leftFront.setPower(.69);
            rightFront.setPower(.76);
        }
        //DriveRight
        if (leftBackTicks > 0 && leftFrontTicks < 0 && rightBackTicks < 0 && rightFrontTicks > 0) {
            leftBack.setPower(.28);
            rightBack.setPower(.75);
            leftFront.setPower(.95); //was 80
            rightFront.setPower(.95); // was 85
        }



        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
            idle();
        }
    }
    public void driveLeft(int Ticks) {drive(-Ticks, Ticks, Ticks, -Ticks);}
    public void driveRight(int Ticks) {drive(Ticks, -Ticks, -Ticks, Ticks);}
    public void driveForward(int Ticks) {drive(-Ticks, -Ticks, -Ticks, -Ticks);}
    public void driveBackward(int Ticks) {drive(Ticks, Ticks, Ticks, Ticks);}
    public void turnLeft (int degrees) {int Ticks = degrees * 70 / 6; drive(Ticks, -Ticks, Ticks, -Ticks); }
    public void turnRight (int degrees) {int Ticks = degrees * 70 / 6; drive (-Ticks, Ticks, -Ticks, Ticks); }

    @Override
    public void runOpMode() {

        init(hardwareMap);
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //pak pixels en wacht
        servoGripper.setPosition(0);
        servoIntake.setPosition(1);
        sleep(100);

        //1400 Ticks = 1 Tile(60cm)
        //Rijd naar rechts en naar het bord
        driveLeft(1400);
        driveBackward(2000);

        //uit time based gehaald
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        init(hardwareMap);

        waitForStart();
        runtime.reset();

        arm1.setPower(power);
        sleep(550);

        arm1.setPower(0);
        sleep(200);

       //gele pixel op bord plaatsen
        servoMoveGripper.setPosition(0);
        sleep(800);

        //laat gele pixel los
        servoGripper.setPosition(1);
        sleep(800);

    }
}