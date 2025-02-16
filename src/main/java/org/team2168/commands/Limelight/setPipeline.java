package org.team2168.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2168.subsystems.Limelight;

public class setPipeline extends Command {
    
    private Limelight limelight;
    private int pipeline;
    private boolean isPipelineSet = false;
    double limeErrorTolerance = 1.0; //in degrees
    public setPipeline(Limelight limelight, int pipeline) {

        this.limelight = limelight;
        this.pipeline = pipeline;


    }

    @Override
    public void initialize() {
        //limelight.enableBaseCameraSettings();
    }

    @Override
    public void execute() {
        limelight.setPipelineIndex(null, pipeline); //set limelight name
        isPipelineSet = true;
    } 
            

    public boolean isFinished() {
        return isPipelineSet;
    }
}