package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Pedro.Constants.*;
//import static org.firstinspires.ftc.teamcode.Constants.Game.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "AutoBio")
public class AutoBio extends OpMode{
    @Override
    public void init(){

    }
    private enum AutonomousState {
        START,
        MOVE,
        STOP
    }
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isMoving = false;
    private AutonomousState autonomousState= AutonomousState.START;
    private Pose2D currentPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private Pose2D targetPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public void run() {
        switch (autonomousState) {
            case START:
                targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0, //X --> Forward, -X --> Backward
                        currentPos.getY(DistanceUnit.INCH) - 12.5, AngleUnit.DEGREES, 0); //Y --> Right, -Y --> Left
                autonomousState = AutonomousState.MOVE;
                break;
            case MOVE:
                currentPos = robot.getPinpointPosition();
                double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
                double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));
                double distance = Math.sqrt(dX*dX + dY*dY);
                if (runtime.seconds() > 2) {
                    autonomousState = AutonomousState.STOP;
                }
            case STOP:
                robot.stop();
                isMoving = false;
                //finished = true;
                break;
        }
    }

    @Override
    public void loop(){

    }
}
