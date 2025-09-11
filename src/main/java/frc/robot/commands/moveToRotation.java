package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

public class moveToRotation extends Command{
    private final Swerve s_Swerve;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController moveXController = new PIDController(2, 0, 0);//(2.1, 0, 0);
    private final PIDController moveYController = new PIDController(2, 0, 0);//(2.1, 0, 0);
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private boolean isDone;
    private double thetaGoal;
    private double y;
    private long curr_tag_in_view;
    public moveToRotation(
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider) {
        this.s_Swerve= s_Swerve;
        this.poseProvider = poseProvider;
        // this.amount_offset = amount_offset;
        // moveXController.setTolerance(0.05);
        // moveYController.setTolerance(0.05);
        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
        isDone = false;
        s_Swerve.togglePreciseTargeting(true);
        curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);

        if (curr_tag_in_view < 0){
            isDone = true;
            System.out.println("No apriltag");
        }
        else{
            var roboPose = poseProvider.get();
            Pose2d tag_pose = new Pose2d();
            if (curr_tag_in_view > 0){
                tag_pose = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d();

            }
            Rotation2d tag_theta = tag_pose.getRotation().plus(Rotation2d.fromDegrees(180));
            double diff_t = roboPose.getRotation().minus(tag_theta).getDegrees();
            System.out.println("t_dff" + diff_t + "angle tag" + tag_theta + "robto angle: " + roboPose.getRotation().getDegrees());

            thetaGoal = tag_theta.getDegrees();


        }
        
        
    }
  
    @Override
    public void execute() {
        System.out.println("Executed main loop: "+ isDone);
        var robotPose2d = poseProvider.get();
        if (isDone){
            return;
        }
        double delt =thetaGoal - robotPose2d.getRotation().getDegrees();
        System.out.println("ID: "+ curr_tag_in_view + " diff x: " + delt);
        // Output Volts is capped at 2 to prevent brownout
        double tOutput = Math.min(moveXController.calculate(-1*delt), 3);
        s_Swerve.drive(new Translation2d(0, 0),tOutput , true, true);
        if (Math.abs(delt) < 10){
            isDone = true;
        }
        else{
            isDone = false;
        }
        
    }
    @Override
    public boolean isFinished(){
        return isDone;
    }
    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
    }
    
}