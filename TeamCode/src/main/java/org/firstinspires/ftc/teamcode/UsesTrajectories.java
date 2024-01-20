package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.TreeMap;

public interface UsesTrajectories {
    public enum trajNames{}
    //public TreeMap<AMLAutoBlue.trajNames, TrajectorySequence> trajs = new TreeMap<>();

    public void initTrajMap();
    public Pose2d switchCurPose(Pose2d newPose);
}
