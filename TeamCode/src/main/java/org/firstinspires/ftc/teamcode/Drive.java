package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class Drive extends OpMode {
    private final Robot robot = new Robot();
    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.setFE(0.24);
    }
    @Override
    public void loop(){
        if(gamepad1.dpadUpWasPressed()){
            robot.setFE(robot.getFE()-0.01);
        }
        if(gamepad1.dpadDownWasPressed()){
            robot.setFE(robot.getFE()+0.01);
        }
        if(gamepad1.yWasPressed()){
            robot.setFE(0.24);
        }
        if(gamepad1.aWasPressed()){
            robot.setFE(0.77);
        }

        double drive  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        robot.move(drive, strafe, twist, time);

        telemetry.addData("Right Actuator Position",robot.getFE());
    }
}
