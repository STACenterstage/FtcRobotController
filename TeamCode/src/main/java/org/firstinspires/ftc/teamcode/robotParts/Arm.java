package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;


public class   Arm {

    public DcMotorEx arm;

    public DcMotorEx spoel;

    private Servo servoChopstickL;
    private Servo servoChopstickR;

    private Servo servoIntakeL;
    private Servo servoIntakeR;

    private Servo servoMoveGripper;
    private Servo servoIntake;
    public Servo servoVliegtuig;
    public Servo servoVliegtuigHouder;
    public Servo servoGripperHold;

    double lowerLimit = -600;
    double upperLimit = 4500;

    public void init(HardwareMap map) {
        arm = map.get(DcMotorEx.class, "arm1");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spoel = map.get(DcMotorEx.class, "spoel");

        servoChopstickL = map.get(Servo.class, "servoChopstickL");
        servoChopstickR = map.get(Servo.class, "servoChopstickR");
        servoMoveGripper = map.get(Servo.class, "servoMoveGripper");
        servoIntake = map.get(Servo.class, "Intake");
        servoVliegtuig = map.get(Servo.class, "Vliegtuig");
        servoVliegtuigHouder = map.get(Servo.class, "VliegtuigHouder");
        servoGripperHold = map.get(Servo.class, "GripperHold");

        servoIntakeL = map.get(Servo.class, "servoIntakeL");
        servoIntakeR = map.get(Servo.class, "servoIntakeR");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void hold(){double power = arm.getPower(); arm.setPower(power);}

    //public void spoelPositivePower(double spoelPower){spoel.setPower(spoelPower);}
    //public void spoelNegativePower(double spoelPower){spoel.setPower(-spoelPower);}

    
    public int ArmPos(){return -spoel.getCurrentPosition();}

    public void intakeL(double position){servoIntakeL.setPosition(position);}
    public void intakeR(double position){servoIntakeR.setPosition(position);}

    public void move(double armPowerLocal){arm.setPower(armPowerLocal);}
    public void chopstickL(double position){servoChopstickL.setPosition(position);}
    public void chopstickR(double position){servoChopstickR.setPosition(position);}
    public void moveGripper(double position){servoMoveGripper.setPosition(position);}
    public void servoIntake(double position){servoIntake.setPosition(position);}
    public void servoVliegtuig(double position){servoVliegtuig.setPosition(position);}
    public void servoVliegtuigHouder(double position){servoVliegtuigHouder.setPosition(position);}
   // public void servoGripperHold (double position){servoGripperHold.setPosition(position);}
   /*public double goToHeight(int position, Telemetry telemetry) {
       double margin = 50.0;
       double currentPosLeft = arm.getCurrentPosition();
       double distance = Math.abs(currentPosLeft - position);
       if (currentPosLeft < position) {
           if (distance > margin) {
               arm.setPower(-1);
           } else {
               arm.setPower(-1 * (distance/margin) * 0.4);
           }
           telemetry.addLine("up");
       } else if (currentPosLeft > position) {
           if (distance > margin) {
               arm.setPower(1);
           } else {
               arm.setPower(1 * (distance/margin) * 0.4);
           }
           telemetry.addLine("down");
       } else if (position == 0 && currentPosLeft <= 0) {
           arm.setPower(0);
       } else {
           arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       }
       return distance;
   }
    public void updateArm(boolean btns, double power, int height, Telemetry telemetry) {
        double distance = 0;
        if (btns) {
            distance = goToHeight(height, telemetry);
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("arm goal", height);
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("arm power", arm.getPower());
        } else {
            int position = arm.getCurrentPosition();

            if (position <= lowerLimit && power <= 0) {
                arm.setPower(0);
            }
            else if (position >= upperLimit && power >= 0) {
                arm.setPower(0);
            } else {
                arm.setPower(power);
            }
            telemetry.addData("arm", position);
            telemetry.addData("arm power", arm.getPower());
            telemetry.addData("distance to goal", distance);
            }

        }*/
    }
