package org.frc5687.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.OutliersContainer;


public class Lights extends OutliersSubsystem{

    private final CANdle _candle;
    private final CANdleConfiguration _config;
    private Animation _animate;
    private AnimationType _currentAnimation;
    private int[] _color;
    private double _brightness;
    private boolean _debugLightsEnabled = false;

    private boolean _dirty  = true;

    public Lights(OutliersContainer _container) {
        super(_container);
        _candle = new CANdle(RobotMap.CAN.CANDLE.PORT, "CANivore");
        _config = new CANdleConfiguration();
        // Set LED strip type
        _config.stripType = LEDStripType.RGB;
        // Sets LED brightness
        setBrightness(1);
        _config.brightnessScalar = _brightness;
        _candle.configAllSettings(_config);
        // set the _color in the constructor to make sure it's not null;
        _color = Constants.CANdle.LEAF00;
        setColor(Constants.CANdle.RUFOUS);
        switchAnimation(AnimationType.STATIC);
    }

    // Set the color of the lights

    public void setColor(int[] color) {
        if (_color != null &&  !colorsMatch(color, _color))  {
            _color = color;
            _dirty = true;
        }
    }

    public int[] getColor() {
        return _color;
    }

    private boolean colorsMatch(int[] color1, int[] color2) {
        if (color1[0] != color2[0]) { return false;}
        if (color1[1] != color2[1]) { return false;}
        if (color1[2] != color2[2]) { return false;}
        return true;
    }

    public void setBrightness(double brightness) {
        if (Math.abs(_brightness - brightness) > 0.0001) {
            _brightness = brightness;
            _dirty = true;
        }
    }
    public double getBrightness() {
        return _brightness;
    }

    

    /**
     * Switch the current animation to the parameter.
     *
     * @param animation
     */
    public void switchAnimation(AnimationType animation) {
        
        if (_currentAnimation == animation && !_dirty) {
            return;
        }
        _currentAnimation = animation;
        switch (animation) {
            case COLOR_FLOW:
                _animate =
                        new ColorFlowAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                ColorFlowAnimation.Direction.Forward);
                break;
            case FIRE:
                _animate =
                        new FireAnimation(
                                _brightness,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                0.5,
                                0.5);
                break;
            case RAINBOW:
                _animate =
                        new RainbowAnimation(
                                1.0, Constants.CANdle.SPEED, Constants.CANdle.NUM_LED);
                break;
            case STROBE:
                _animate =
                        new StrobeAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED);
                break;
            case LARSON:
                _animate = new LarsonAnimation(_color[0], _color[1], _color[2]);
                break;
            case RGB_FADE:
                _animate =
                        new RgbFadeAnimation(
                                _brightness, Constants.CANdle.SPEED, Constants.CANdle.NUM_LED);
                break;
            case SINGLE_FADE:
                _animate =
                        new SingleFadeAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED);
                break;
            case TWINKLE:
                _animate =
                        new TwinkleAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                Constants.CANdle.TWINKLEPERCENT);
                break;
            case TWINKLE_OFF:
                _animate =
                        new TwinkleOffAnimation(
                                _color[0],
                                _color[1],
                                _color[2],
                                0,
                                Constants.CANdle.SPEED,
                                Constants.CANdle.NUM_LED,
                                Constants.CANdle.TWINKLEOFFPERCENT);
                break;
            case STATIC:
                _animate = null;
                break;
        }
        _dirty = true;
    }

    /** Has all the logic for the lights, and updates the CANdle with animations and static colors. */
    @Override
    public void periodic() {
        if (!_dirty) {
            return;
        }

        _candle.configBrightnessScalar(_brightness);
        
        if (_animate == null) {
            _candle.clearAnimation(0); // very important
            _candle.setLEDs(_color[0], _color[1], _color[2]);
        } else {
            _candle.animate(_animate, 0);
        } 
        _dirty = false;

    } 

    public void setDebugLightsEnabled(boolean value) {
        _debugLightsEnabled = value;
    }

    public boolean getDebugLightsEnabled() {
        return _debugLightsEnabled;
    }

    public void updateDashboard() {}

    public enum AnimationType {
        COLOR_FLOW(0),
        FIRE(1),
        RAINBOW(2),
        STROBE(3),
        LARSON(4),
        RGB_FADE(5),
        SINGLE_FADE(6),
        TWINKLE(7),
        TWINKLE_OFF(8),
        STATIC(9);

        private int _value;

        AnimationType(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}

