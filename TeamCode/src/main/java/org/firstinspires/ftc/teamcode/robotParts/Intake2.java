/*package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake", group = "TeleOp")

abstract public class Intake2 extends LinearOpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    public void inituinitu() {
        motor1 = hardwareMap.get(DcMotor.class, "Intake_Left");
        motor2 = hardwareMap.get(DcMotor.class, "Intake_Right");
    }

    public void loopuduloopudu() {
        if (gamepad2.left_bumper) {
            motor1.setPower(1.0);
            motor2.setPower(-1.0);
        } else if (gamepad2.right_bumper) {
            motor1.setPower(-1.0);  // Stop motor1
            motor2.setPower(1.0); // Set motor2 to full power reverse
        } else {
            // Neither bumper is pressed, stop both motors
            motor1.setPower(0.0);
            motor2.setPower(0.0);
        }
    }
}
*/