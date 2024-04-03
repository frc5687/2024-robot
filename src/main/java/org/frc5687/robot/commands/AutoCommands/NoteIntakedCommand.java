package org.frc5687.robot.commands.AutoCommands;

import org.frc5687.robot.RobotState;
import org.frc5687.robot.commands.OutliersCommand;

public class NoteIntakedCommand extends OutliersCommand{
    private int _id;
    private boolean _noteIntaked;
    private RobotState _robotState = RobotState.getInstance();

    public NoteIntakedCommand(int id, boolean NoteIntaked)
        {
        _id = id;
        _noteIntaked = NoteIntaked;
        

    }
    
    @Override
    public void initialize() {
        _robotState.setIntakedNote(_id, _noteIntaked);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
