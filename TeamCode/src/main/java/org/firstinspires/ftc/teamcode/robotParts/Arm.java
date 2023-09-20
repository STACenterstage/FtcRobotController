package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    DcMotorEx arm;

    public void init(HardwareMap map) {
        arm = map.get(DcMotorEx.class, "arm1");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void move(double armPowerLocal) {
        arm.setPower(armPowerLocal);
    }
}