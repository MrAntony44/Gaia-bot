// Solar Panel Movement - Servo use
@TeleOp(name="driveJava", group="Pushbot")
public class DriveJava extends LinearOpMode {
    Servo servo
    Servo solar_Panel_Thing

    public void autoMode(){
        while(gamepad1.dpad_down){ // replace gamepad1.dpad_down with boolean that stores true if robot is at deployment area.
            servo.setPosition(1);
            pause(500);
            servo.setPosition(0);
        }
    }
}