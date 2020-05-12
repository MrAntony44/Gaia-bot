public class DriveJava extends LinearOpMode {
    private DcMotor wind_turbine - null;

    public void runOpMode(){
        wind_turbine = hardwareMap.dcMotor.get( "wind_turbine");
        wind_turbine.setDirection(DcMotor.Direction.FORWARD);
    }
    public void drive(){ //Name typical
        while(gamepad1.x){ // redefine contoller input for 
            wind_turbine.setPower(1);
            pause(1500);
            wind_turbine.setPower(0);
        }
    }
}