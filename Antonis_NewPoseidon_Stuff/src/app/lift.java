private servo
private DcMotor lift_thing = null;
Servo lift_servo1;
Servo lift_servo2;

public class lift {
    lift_servo2 = hardwareMap.get(Servo.class , "lift_servo2"); //I think we need those
    lift_servo1 = hardwareMap.get(Servo.class , "lift_servo1");
    lift_servo1.setDirection(Servo.Direction.REVERSE); //This one too

    while(gamepad.dpad_right){ //Button Bind
        lift_servo1.setPosition(1);
        lift_servo2.setPosition(1);
    }



}