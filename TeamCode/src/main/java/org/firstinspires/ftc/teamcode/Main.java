package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Test", group="Iterative Opmode")
public class main extends LinearOpMode {
    DcMotorEx[] driveMotors = new DcMotorEx[4];
    double maxSpeed = 1000.0;
    double rotationMaxSpeed = 500.0;

    DcMotorEx liftTest;

    private void init_dcmotor(){
        //Przypisanie silnikow do portow
        driveMotors[0]=hardwareMap.get(DcMotorEx.class,"motorTest0");
        driveMotors[1]=hardwareMap.get(DcMotorEx.class,"motorTest1");
        driveMotors[2]=hardwareMap.get(DcMotorEx.class,"motorTest2");
        driveMotors[3]=hardwareMap.get(DcMotorEx.class,"motorTest3");
        //Ustawienie "mode"
        for (int i=0; i<4;i++){
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        driveMotors[0].setDirection(DcMotor.Direction.FORWARD);
        driveMotors[1].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotor.Direction.FORWARD);

    }

    private void init_lifTest(){
        liftTest = hardwareMap.get(DcMotorEx.class ,"liftTest");
        liftTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTest.setDirection((DcMotor.Direction.FORWARD));
    }
    public void steerDriveMotors(){
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

    public void liftMotor(){
        double liftSpeed = 200.0;

        if(gamepad1.y){
            liftTest.setVelocity(liftSpeed);
        }else if(gamepad1.a){
            liftTest.setVelocity(-liftSpeed);
        }else{
            liftTest.setVelocity(0);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        init_dcmotor();
        init_lifTest();
        telemetry.addData("Status", "initialized");
        telemetry.update();


        Thread thread0 = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    steerDriveMotors();
            }
        });

        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true)
                    liftMotor();
            }
        });

        thread0.start();
        thread1.start();

        thread0.join();
        thread1.join();

        waitForStart();
        while(true){


        }
    }
}
