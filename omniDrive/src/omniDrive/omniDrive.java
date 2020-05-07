package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;

@TeleOp(name="driveJava", group="Pushbot")
public class DriveJava extends LinearOpMode {
    
    
    private VuforiaFG2019 vuforiaFG2019;

    VuforiaBase.TrackingResults vuforiaResult;
    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;

    BNO055IMU imu;
    @Override
    public void runOpMode()  {
        vuforiaFG2019 = new VuforiaFG2019();
 
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");


        front_left_wheel.setDirection(DcMotor.Direction.REVERSE); 
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

       // vuforiaFG2019 = new Vuforia2019();
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //lift_thing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //lift_thing.setVelocity(300);
        imu.initialize(parameters);
      //  vuforiaFG2019.initialize(hardware.get(WebcamNameWebcamName.class, "Webcam 1"),false,true,0,0,0,-90,180,0);
        while(!opModeIsActive()){}
        
        while(opModeIsActive()){
            drive();
            resetAngle();
            vuforiaFG2019.activate();
            //driveSimple();
            telemetry.update();
        }
        
    }

    public void drive(){
         double Protate = gamepad1.right_stick_x/4;
         double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
         double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
         double theta = 0;
         double Px = 0;
         double Py = 0;
         boolean state = false;
         double i = 0;
         double lift = 0;

         double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
                if (gyroAngle <= 0) { 
            gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
            }
            gyroAngle = -1 * gyroAngle;
        
    if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly. 
        gyroAngle = -Math.PI/2;
        }
    

    theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
    Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
    Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));
    telemetry.addData("Stick_X", stick_x);
    telemetry.addData("Stick_Y", stick_y);
    telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
    telemetry.addData("Front Left", Py - Protate);
    telemetry.addData("Back Left", Px - Protate);
    telemetry.addData("Back Right", Py + Protate);
    telemetry.addData("Front Right", Px + Protate);
    
    front_left_wheel.setPower(Py - Protate);
    back_left_wheel.setPower(Px - Protate);
    back_right_wheel.setPower(Py + Protate);
    front_right_wheel.setPower(Px + Protate);
}
public void resetAngle(){
    if(gamepad1.a){
        reset_angle = getHeading() + reset_angle;
        }
    }
    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }
}