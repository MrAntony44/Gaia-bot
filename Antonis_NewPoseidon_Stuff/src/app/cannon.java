private DcMotor shooter_thing = null;
private DcMotor front_left_wheel = null;
private DcMotor back_left_wheel = null;
private DcMotor back_right_wheel = null;
private DcMotor front_right_wheel = null;

public class shoot {
    front_left_wheel.setPower(0); //To stop all movement
    back_left_wheel.setPower(0);  //To stop all movement
    back_right_wheel.setPower(0);//To stop all movement
    front_right_wheel.setPower(0);//To stop all movement
        shooter_thing.setPower(-0.2);
        pause(1000);
        shooter_thing.setPower(-0.4);
        pause(1000);
        shooter_thing.setPower(-0.5);
        pause(1000);
        shooter_thing.setPower(-0.6);
        pause(1000);
        shooter_thing.setPower(-0.785);
}