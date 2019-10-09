package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

@TeleOp(name="My Op Mode", group="Iterative Opmode")
public class MyMainClass extends LinearOpMode {

    private DistanceSensor imu;
    private DigitalChannel touchSensor;
    private DcMotorEx []dcMotor = new DcMotorEx[4];
    private double motorSpeedX;
    private double motorSpeedY;
    private double motorSpeedX1;
    private double motorSpeedY1;
    private HashMap sensorData = new HashMap();

    private void initDcMotor(String motorName){
        for(int i = 0; i < 4; i++) {
            dcMotor[i] = hardwareMap.get(DcMotorEx.class, motorName + Integer.valueOf(i).toString() );
            dcMotor[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            switch (i % 2) {
                case 0:
                    dcMotor[i].setDirection(DcMotorEx.Direction.FORWARD);
                    break;
                case 1:
                    dcMotor[i].setDirection(DcMotorEx.Direction.REVERSE);
                    break;
            }
        }
    }

    public void steerJoystick(){
        for(int i = 0; i < 2; i++){
            motorSpeedX = gamepad1.left_stick_x * 2600;
            motorSpeedY = gamepad1.left_stick_y * 2600;

            motorSpeedX1 = motorSpeedX;
            motorSpeedY1 = motorSpeedY;

            if(gamepad1.left_bumper){
                motorSpeedX1 -= 700;
                motorSpeedX += 700;
                if (motorSpeedX > 2600) {
                    motorSpeedX = 2600;
                }
            }
            if(gamepad1.right_bumper){
                motorSpeedX1 -= 700;
                motorSpeedX += 700;
                if (motorSpeedX > 2600){
                    motorSpeedX = 2600;
                }
            }

            if(i == 0){
                dcMotor[i].setVelocity(motorSpeedX1);
                dcMotor[i+2].setVelocity(-motorSpeedX);
            }
            else{
                dcMotor[i].setVelocity(motorSpeedY1);
                dcMotor[i+2].setVelocity(-motorSpeedY);
            }

        }
        telemetry.update();
    }

    private void initDistanceSensor(String sensorName){
        imu = hardwareMap.get(DistanceSensor.class, sensorName);
        sensorData.put("distance", 0.0);
    }

    private void initTouchSensor(String sensorName){
        touchSensor = hardwareMap.get(DigitalChannel.class ,sensorName);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        sensorData.put("touch", false);
    }

    //puts sensor data in the array
    public void getSensorData(){
        sensorData.put("distance", imu.getDistance(DistanceUnit.CM));
        sensorData.put("touch", touchSensor.getState());
    }

    //transmits data to the driver station
    public void showSensorData(){
        telemetry.addData("Distance Sensor: ", sensorData.get("distance"));

        if((boolean)(sensorData.get("touch"))){
            telemetry.addData("Touch Sensor: ", "RELEASED");
        }
        else{
            telemetry.addData("TouchSensor", "PRESSED");
        }
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //inits (no shit Sherlock):
        //initDistanceSensor("distanceTest");
        //initTouchSensor("touchTest");
        initDcMotor("motorTest");

        telemetry.addData("status", "Initialized");
        waitForStart();

        //main loop:
        while(opModeIsActive()){
            // getSensorData();
            // showSensorData();
            steerJoystick();

        }
    }
}