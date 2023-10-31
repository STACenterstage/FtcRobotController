package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoControlWithGamepad", group = "TeleOp")
public class Outtake extends LinearOpMode {

    private Servo servoGripper;
    private Servo servoMoveGripper;
    private DcMotor arm1;

    private double servoGripperMin = 0.0;  // Minimum position for servo1 (in degrees)
    private double servoGripperMax = 180.0;  // Maximum position for servo1 (in degrees)
    private double servoMoveGripperMin = 0.0;  // Minimum position for servo2 (in degrees)
    private double servoMoveGripperMax = 180.0;  // Maximum position for servo2 (in degrees)


    @Override
    public void runOpMode() {
        servoGripper = hardwareMap.get(Servo.class, "servoGripper"); // Replace "servo1" with your servo name
        servoMoveGripper = hardwareMap.get(Servo.class, "servoMoveGripper"); // Replace "servo2" with your servo name
        arm1 = hardwareMap.get(DcMotor.class, "arm1"); // Replace "motor" with your motor name

        waitForStart();

        while (opModeIsActive()) {
            // Read the motor position in ticks
            double motorPosition = arm1.getCurrentPosition();

            // Map the motor position to servo positions
            double servo1Pos = map(motorPosition, 0.0, 1023.0, servoGripperMin, servoGripperMax);
            double servo2Pos = map(motorPosition, 0.0, 1023.0, servoMoveGripperMin, servoMoveGripperMax);

            // Set the servo positions based on gamepad input
            servoGripper.setPosition(gamepad2.left_stick_x); // Use left stick X for servo1
            servoMoveGripper.setPosition(gamepad2.right_stick_x); // Use right stick X for servo2

            // Your other robot logic goes here

            idle();
        }
    }

    // Custom map function for doubles
    private double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}
