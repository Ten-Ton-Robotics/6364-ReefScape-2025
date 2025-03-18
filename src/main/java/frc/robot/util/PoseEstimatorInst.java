// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.Drivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PhotonVisionHandler;

public class PoseEstimatorInst {
    private PhotonVisionHandler visionHandler;
    private Optional<EstimatedRobotPose> prevVisionOut;
    private Optional<EstimatedRobotPose> VisionOut;
    private CommandSwerveDrivetrain drivetrain;
    private Field2d m_VisionPose;
    private boolean isSimulation;

    public PoseEstimatorInst(PhotonVisionHandler visionHandler, CommandSwerveDrivetrain drivetrain, Field2d m_VisionPose) {
        this.visionHandler = visionHandler;
        this.drivetrain = drivetrain;
        this.m_VisionPose = m_VisionPose;
        this.isSimulation = Utils.isSimulation();
        this.prevVisionOut = Optional.empty();
    }

    public void updatePose(boolean reef) {
        if (visionHandler == null) {
            System.err.println("VisionHandler is null. Skipping pose estimation update.");
            return;
        }

        if (prevVisionOut.isPresent()) {
            VisionOut = visionHandler.getEstimatedGlobalPose(prevVisionOut.get().estimatedPose.toPose2d());
        } else {
            VisionOut = visionHandler.getEstimatedGlobalPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        }

        prevVisionOut = VisionOut;

        try {
            if (VisionOut.isPresent()) {
                final Pose2d visPose = VisionOut.get().estimatedPose.toPose2d();
                final double posDiff = drivetrain.getPoseDifference(visPose);
                final List<PhotonTrackedTarget> tags = VisionOut.get().targetsUsed;

                // Set and put output from Vision on Smart Dashboard for debugging
                m_VisionPose.setRobotPose(VisionOut.get().estimatedPose.toPose2d());
                // SmartDashboard.putData("Vision Pose", m_VisionPose);

                if (tags.size() < 1) {
                    return;
                }

                double lateralDeviation, angularDeviation;

                // Logic for vision pose estimation based on tags and area
                if (tags.size() > 1 && visionHandler.avgTagArea(tags) > 0.8) {
                    lateralDeviation = 0.5;
                    angularDeviation = 6; //6;
                } else if (tags.get(0).getArea() > 0.8) {
                    lateralDeviation = 1.0;
                    angularDeviation = 12; //12;
                } else if (tags.get(0).getArea() > 0.1) {
                    lateralDeviation = 2.0;
                    angularDeviation = 30; //30;
                } else {
                    return;
                }

                // Only fuse with WPIlib Kalman filter when the simulation is off
                Pose2d visPose2d = VisionOut.get().estimatedPose.toPose2d();
                double visionstamp = VisionOut.get().timestampSeconds;
                this.drivetrain.addVisionMeasurement(visPose2d, visionstamp, VecBuilder.fill(lateralDeviation, lateralDeviation, Units.degreesToRadians(angularDeviation)));
                
            }

            this.m_VisionPose.setRobotPose(drivetrain.getState().Pose);
            SmartDashboard.putData("Robot Pose", this.m_VisionPose);
            
        } catch (Exception e) {
            System.out.println(e);
        }
    }
}
