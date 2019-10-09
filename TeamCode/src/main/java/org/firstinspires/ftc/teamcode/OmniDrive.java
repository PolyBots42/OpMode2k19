package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OmniDrive extends LinearOpMode {
    private double[] velocities = new double[4];
    private DcMotorEx[] motors = new DcMotorEx[4];

    /*
        0 - front motor
        1 - left motor
        2 - rear motor
        3 - right motor
     */

    public OmniDrive(String name) {

        super();
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, name + Integer.toString(i));
            motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        for(int i = 0 ; i < 2 ; i++) {
            motors[i].setDirection(DcMotorEx.Direction.FORWARD);
        }
        for(int i = 2 ; i < 4 ; i++) {
            motors[i].setDirection(DcMotorEx.Direction.REVERSE);
        }
    }

    public void drive(double speedX, double speedY, double rotSpeed){

        velocities[0] = velocities[2] = speedX;
        velocities[1] = velocities[3] = speedY;

        for(int i = 0 ; i < 2 ; i++){
            velocities[i] += rotSpeed;
        }
        for(int i = 2 ; i < 4 ; i++){
            velocities[i] -= rotSpeed;
        }

        for(int i = 0 ; i < 4 ; i++){
            if(velocities[i] < -2600.0){
                velocities[i] = -2600.0;
            }
            else if(velocities[i] > 2600.0){
                velocities[i] = 2600.0;
            }

            motors[i].setVelocity(velocities[i]);
        }
    }

    public void reverseDirections(){
        for(int i = 0 ; i < 2 ; i++) {
            motors[i].setDirection(DcMotorEx.Direction.REVERSE);
        }
        for(int i = 2 ; i < 4 ; i++) {
            motors[i].setDirection(DcMotorEx.Direction.FORWARD);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

