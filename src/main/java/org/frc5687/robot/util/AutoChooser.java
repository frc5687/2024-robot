package org.frc5687.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChooser extends OutliersProxy {

    private SendableChooser<Node> _firstNodeChooser;
    private Node _firstNode;
    private AutoType _autoType;
    private SendableChooser<AutoType> _autoTypeChooser;

    public AutoChooser() {
        _firstNodeChooser = new SendableChooser<>();
        _firstNodeChooser.setDefaultOption("One", Node.OneCone);
        _firstNodeChooser.addOption("Two", Node.TwoCube);
        _firstNodeChooser.addOption("Three", Node.ThreeCone);
        _firstNodeChooser.addOption("Four", Node.FourCone);
        _firstNodeChooser.addOption("Five", Node.FiveCube);
        _firstNodeChooser.addOption("Six", Node.SixCone);
        _firstNodeChooser.addOption("Seven", Node.SevenCone);
        _firstNodeChooser.addOption("Eight", Node.EightCube);
        _firstNodeChooser.addOption("Nine", Node.NineCone);
        SmartDashboard.putData("First Node", _firstNodeChooser);

        _autoTypeChooser = new SendableChooser<>();
        _autoTypeChooser.setDefaultOption("Select An Auto!", AutoType.NoAuto);
        _autoTypeChooser.addOption("Drive For Time", AutoType.DriveForTime);
        _autoTypeChooser.addOption("DriveToLevel", AutoType.DrivetoLevel);
        _autoTypeChooser.addOption("One And A Half Level", AutoType.OneAndAHalfLevel);
        _autoTypeChooser.addOption("Two Piece", AutoType.TwoPiece);
        _autoTypeChooser.addOption("Two and A Half Piece", AutoType.TwoAndAHalfPieceAuto);
        _autoTypeChooser.addOption("Two and A Half Piece Level", AutoType.TwoAndAHalfPieceLevelAuto);
        _autoTypeChooser.addOption("Steal Cubes", AutoType.StealCubes);
        _autoTypeChooser.addOption("Three Cube Level", AutoType.ThreeCubeLevel);
        _autoTypeChooser.addOption("Three Cube No Level", AutoType.ThreeCubeNoLevel);
        _autoTypeChooser.addOption("Three Piece", AutoType.ThreePiece);
        SmartDashboard.putData("Auto Type", _autoTypeChooser);
    }

    public void updateChooser() {
        _firstNode = _firstNodeChooser.getSelected();
        _autoType = _autoTypeChooser.getSelected();
        metric("First Piece", _firstNode.name());
        metric("Auto Type", _autoType.name());
    }

    public Node getFirstNode() {
        return _firstNode;
    }

    public AutoType getAutoType() {
        return _autoType;
    }

    @Override
    public void updateDashboard() {}

    // public enum Piece {
    //     Unknown(-1),
    //     Cone(0),
    //     Cube(1);

    //     private int _value;

    //     Piece(int value) {
    //         _value = value;
    //     }

    //     public int getValue() {
    //         return _value;
    //     }
    // }

    public enum Node {
        Unknown(-1),
        OneCone(0),
        TwoCube(1),
        ThreeCone(2),
        FourCone(3),
        FiveCube(4),
        SixCone(5),
        SevenCone(6),
        EightCube(7),
        NineCone(8);

        private int _value;

        Node(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public enum AutoType {
        Unknown(-1),
        DriveForTime(0),
        DrivetoLevel(1),
        TwoPiece(2),
        TwoAndAHalfPieceAuto(3),
        TwoAndAHalfPieceLevelAuto(4),
        StealCubes(5),
        ThreeCubeLevel(6),
        ThreeCubeNoLevel(7),
        OneAndAHalfLevel(8),
        ThreePiece(9),
        NoAuto(10);

        private int _value;

        AutoType(int value) {
            value = _value;
        }

        public int getValue() {
            return _value;
        }
    }
}
