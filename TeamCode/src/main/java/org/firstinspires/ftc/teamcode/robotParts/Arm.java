package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    DcMotorEx arm;
    private Servo servoGripper;
    private Servo servoMoveGripper;
    private Servo servoIntake;

    public void init(HardwareMap map) {
        arm = map.get(DcMotorEx.class, "arm1");

        servoGripper = map.get(Servo.class, "servoGripper");
        servoMoveGripper = map.get(Servo.class, "servoMoveGripper");
        servoIntake = map.get(Servo.class, "Intake");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void move(double armPowerLocal){arm.setPower(armPowerLocal);}
    public void gripper(double position){servoGripper.setPosition(position);}
    public void moveGripper(double position){servoMoveGripper.setPosition(position);}
    public void servoIntake(double position){servoIntake.setPosition(position);}



}
