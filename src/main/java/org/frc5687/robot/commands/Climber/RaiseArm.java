// package org.frc5687.robot.commands.Climber;

// import org.frc5687.robot.Constants;
// import org.frc5687.robot.commands.OutliersCommand;
// import org.frc5687.robot.subsystems.Climber;
// import org.frc5687.robot.subsystems.Climber.ClimberStep;

// import edu.wpi.first.wpilibj.Relay.Value;

// public class RaiseArm extends OutliersCommand {
    
//     private Climber _climber;
//     private Step _step = Step.START;

//     public RaiseArm (Climber climber) {
//         _climber = climber;
//         addRequirements(_climber);
//         info("Instantiated RaiseArm Check");
//     }

//     @Override
//     public void initialize(){
//         super.initialize();
//         info("Initialized RaiseArm");
//         _climber.setStep(Climber.ClimberStep.RAISE_ARM);
//         _step = Step.START;
//     }

//     @Override
//     public void execute() {
//         super.execute();
//         switch(_step) {
//             case START:
//                 _step = Step.PULL_RATCHET;
//                 info("RaiseArm advancing to PULL_RATCHET step.");
//                 break;
//             case PULL_RATCHET:
//                 _climber.setPositionMeters(Constants.Climber.OPEN_WINCH);
//                 _climber.changeRatchetUp();
//                 _step = Step.RAISE_ARM;
//                 info("RaiseArm advancing to RAISE_ARM step.");
//                 break;
//             case RAISE_ARM:
//                 _climber.setPositionMeters(Constants.Climber.UPPER_LIMIT);
//                 _step = Step.DONE;
//                 info("RaiseArm advancing to RAISE_ARM step.");
//                 break;
//             case DONE:
//                 break;
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         super.isFinished();

//         if ( _step == Step.DONE && {

//         }
//     }

//     public enum Step {
//         START(0),
//         PULL_RATCHET(1),
//         RAISE_ARM(2),
//         DONE(3);

//         private final int _value;
//         Step(int value) { 
//             _value = value; 
//         }

//         public int getValue() { 
//             return _value; 
//         }
//     }
// }
