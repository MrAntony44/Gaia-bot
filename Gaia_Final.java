package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;





@TeleOp(name="RTTR_Final", group="Pushbot")
public class RTTR_Final extends LinearOpMode {
    
    

    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;
    private DcMotor heavy = null;
    private DcMotor wind_turbine = null;
    private DcMotor shooter_thing = null;
    private DcMotor collection_thing = null;
    Servo solar_Panel_Mechanism;
    Servo servo;
    ColorSensor sensorColor;
    Servo up_color;
    Servo down_color;
    Servo treeMovement1;
    Servo treeMovement2;


    BNO055IMU imu;
    @Override
    public void runOpMode()  {
 
        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        
        wind_turbine = hardwareMap.dcMotor.get( "wind_turbine");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        heavy = hardwareMap.get(DcMotorEx.class, "heavy");
        shooter_thing = hardwareMap.dcMotor.get( "shooter_thing");
        collection_thing = hardwareMap.dcMotor.get( "collection_thing");
        
        up_color = hardwareMap.get(Servo.class, "up_color" );
        down_color = hardwareMap.get(Servo.class, "down_color" );
        treeMovement1 = hardwareMap.get(Servo.class, "treeMovement1" );
        treeMovement2 = hardwareMap.get(Servo.class, "treeMovement2" );

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE); 
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD);
        wind_turbine.setDirection(DcMotor.Direction.FORWARD);

        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        heavy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        heavy.setTargetPosition(0);
        heavy.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       
       
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

  
        

        while(!opModeIsActive()){}
        
        while(opModeIsActive()){
      
        
            drive();
            resetAngle();
            telemetry.update();
        }
        
    }

    public static void pause(int ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e){
            System.err.format( "IOException: %s%n" , e);
        }
    }

    public void shoot(){
        //shooter direction...change + to - if you want to change direction
        front_left_wheel.setPower(0);
        back_left_wheel.setPower(0);
        back_right_wheel.setPower(0);
        front_right_wheel.setPower(0);
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
         
               // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

              
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
        (int) (sensorColor.green() * SCALE_FACTOR),
        (int) (sensorColor.blue() * SCALE_FACTOR),
        hsvValues);
        


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
        
        // send the info back to driver station using telemetry function.
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        
        front_left_wheel.setPower(Py - Protate);
        back_left_wheel.setPower(Px - Protate);
        back_right_wheel.setPower(Py + Protate);
        front_right_wheel.setPower(Px + Protate);


        


        //Blue balls
        if (hsvValues[0] >= 170 && hsvValues[0] <= 260){
            up_color.setPosition(1);
        //Green Balls
        }else if (hsvValues[0] >= 75 && hsvValues[0] <= 160){
            up_color.setPosition(0);
        }else{
            up_color.setPosition(0.5);
        }

        while(gamepad1.x){ // redefine contoller input for 
        down_color.setPosition(1);
        }

        while(gamepad1.y){ // redefine contoller input for 
        down_color.setPosition(0);
        }

        


        while(gamepad1.dpad_right){ // Lift
            heavy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            heavy.setTargetPosition(0);
            heavy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        while(gamepad1.x){ // Collection Mechanism - on
            collection_thing.setPower(1);
        }

        while(gamepad1.y){ // Collection Mechanism - off
            collection_thing.setPower(0);
        }
        while(gamepad1.a){ // Shooter
          shoot();
        }
        
        while(gamepad1.dpad_up){ // redefine contoller input for Wind Turbine
            wind_turbine.setPower(1);
            pause(1500);
            wind_turbine.setPower(0);
        }
        while(gamepad1.dpad_down){ // solar panels
            solar_Panel_Mechanism.setPosition(1);
            pause(500);
            solar_Panel_Mechanism.setPosition(0);
        }
        while(gamepad1.left_bumper) { // Tree Movement (servo) - on
            treeMovement1.setPosition(1);
            treeMovement2.setPosition(1);
            
        }
        while(gamepad1.right_bumper){ // Tree Movement - off
            treeMovement1.setPosition(0);
            treeMovement2.setPosition(0);
        }
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


