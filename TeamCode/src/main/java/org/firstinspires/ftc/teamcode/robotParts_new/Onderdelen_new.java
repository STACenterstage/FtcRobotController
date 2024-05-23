package org.firstinspires.ftc.teamcode.robotParts_new;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Onderdelen_new {

    private Servo servo1;
    public void init(HardwareMap map) {
        servo1 = map.get(Servo.class, "servo1");
    }
    public void servo1(double position){servo1.setPosition(position);}
}