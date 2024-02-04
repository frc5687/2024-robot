// package org.frc5687.robot.commands.Climber;

// import org.frc5687.robot.subsystems.Climber;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class AutoClimb {
    
//     private Climber _climber;
    
//     public AutoClimb(Climber climber) {
//         _climber = climber;
//     }

//     @Override
//     public void initialize() {
//         super.initialize();
//         SequentialCommandGroup group = new SequentialCommandGroup();
//         switch(_climber.getStep()) {
//             case UNKNOWN:
//             case STOW:
//             case STOWED:
//                 group.addCommands(new PrepToClimb(_climber));
//                 break;
//             case RAISE_ARM:
//                 group.addCommands(new RaiseArm());
//                 break;            
//             case DONE:
//                 break;
//         }
//         group.schedule();
//     }
// }
