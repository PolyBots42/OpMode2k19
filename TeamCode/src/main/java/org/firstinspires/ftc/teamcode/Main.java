package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Test", group="Iterative Opmode")
public class Main extends LinearOpMode {
    DcMotorEx[] driveMotors = new DcMotorEx[4];
    double maxSpeed = 1000.0;
    double rotationMaxSpeed = 500.0;

    DcMotorEx liftTest;

    private void init_dcmotor(String motorName){
        //DC Motors mapping
        for(int i = 0 ; i < 4 ; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, motorName + Integer.toString(i));
        }
        //Set motors to rotate at set speed
        for (int i=0; i<4;i++){
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        driveMotors[0].setDirection(DcMotor.Direction.FORWARD);
        driveMotors[1].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotor.Direction.FORWARD);

    }

    private void init_lifTest(String motorName){
        //initializing lift DC motor
        liftTest = hardwareMap.get(DcMotorEx.class ,motorName);
        liftTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTest.setDirection((DcMotor.Direction.FORWARD));
    }

    private void init_arm_servos(String servoName){
        // TODO
    }
    private void steer_Drive_Motors(){
        //driving the robot
        double speedX;
        double speedY;

        speedX = gamepad1.left_stick_x * maxSpeed;
        speedY = gamepad1.left_stick_y * maxSpeed;

        if(gamepad1.right_bumper) {
            driveMotors[0].setVelocity(speedX + rotationMaxSpeed);
            driveMotors[1].setVelocity(speedY - rotationMaxSpeed);
            driveMotors[2].setVelocity(speedX - rotationMaxSpeed);
            driveMotors[3].setVelocity(speedY + rotationMaxSpeed);
        }else if(gamepad1.left_bumper){
            driveMotors[0].setVelocity(speedX - rotationMaxSpeed);
            driveMotors[1].setVelocity(speedY + rotationMaxSpeed);
            driveMotors[2].setVelocity(speedX + rotationMaxSpeed);
            driveMotors[3].setVelocity(speedY - rotationMaxSpeed);
        }else{
            driveMotors[0].setVelocity(speedX);
            driveMotors[1].setVelocity(speedY);
            driveMotors[2].setVelocity(speedX);
            driveMotors[3].setVelocity(speedY);
        }
    }

    private void lift_Motor(){
        //controlling the lift
        double liftSpeed = 200.0;

        if(gamepad1.y){
            liftTest.setVelocity(liftSpeed);
        }else if(gamepad1.a){
            liftTest.setVelocity(-liftSpeed);
        }else{
            liftTest.setVelocity(0);
        }

    }

    private void arm_servos(){
        //TODO
    }

    //main function:
    @Override
    public void runOpMode() throws InterruptedException {
        init_dcmotor("motorTest");
        init_lifTest("liftTest");
        telemetry.addData("Status", "initialized");
        telemetry.update();


        Thread drive_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    steer_Drive_Motors();
            }
        });

        Thread lift_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    lift_Motor();
            }
        });

        Thread arm_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    arm_servos();
            }
        });

        drive_thread.start();
        lift_thread.start();
        arm_thread.start();

        drive_thread.join();
        lift_thread.join();
        arm_thread.join();

        waitForStart();
        while(true){

        }
    }
}
