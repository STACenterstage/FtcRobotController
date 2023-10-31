/*package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake", group = "TeleOp")

abstract public class Intake extends LinearOpMode {

    private DcMotorEx IntakeLeft;
    private DcMotorEx IntakeRight;

    public void init(HardwareMap map) {
        IntakeLeft = map.get(DcMotorEx.class, "Intake_Left");
        IntakeRight = map.get(DcMotorEx.class, "Intake_Right");


        while (opModeIsActive()) {
            boolean IntakeOn = gamepad2.left_bumper;
            boolean IntakeOff = gamepad2.right_bumper;

            if(IntakeOn){
                IntakeLeft = IntakeRight;
            } else if (IntakeOff) {
                IntakeLeft = IntakeRight;
            }
        }
    }

    private DcMotor motor1;
    private DcMotor motor2;

    public void inittininty() {
        motor1 = hardwareMap.get(DcMotor.class, "Intake_Left");
        motor2 = hardwareMap.get(DcMotor.class, "Intake_Right");
    }
    public void loopydyloopydy() {
        if (gamepad2.left_bumper) {
            motor1.setPower(1.0);
            motor2.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            motor1.setPower(-1.0);  // Stop motor1
            motor2.setPower(-1.0); // Set motor2 to full power reverse
        } else {
            // Neither bumper is pressed, stop both motors
            motor1.setPower(0.0);
            motor2.setPower(0.0);
        }
    }
}*/
