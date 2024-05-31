package org.firstinspires.ftc.teamcode.robotParts_new;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.Servo;

public class Onderdelen_new {
    private Servo servo0, servo1;
    public void init(HardwareMap map) {
        servo0 = map.get(Servo.class, "servo0");
        servo1 = map.get(Servo.class, "servo1");

    }
    public void servo0(double position){servo0.setPosition(position);}
    public void servo0(Servo.Direction direction){servo0.setDirection(direction);}
    public void servo1(double position){servo1.setPosition(position);}
    public void servo1(Servo.Direction direction){servo1.setDirection(direction);}

}