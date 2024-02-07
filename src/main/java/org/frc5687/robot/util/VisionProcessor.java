package org.frc5687.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import org.zeromq.SocketType;
import org.zeromq.ZMQ;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import org.frc5687.lib.flatbuffers.*;

// Import FlatBuffers generated classes
import com.google.flatbuffers.FlatBufferBuilder;

public class VisionProcessor {
    private final ZMQ.Context context;
    private final Map<String, ZMQ.Socket> subscribers = new HashMap<>();
    private final Map<String, ZMQ.Socket> publishers = new HashMap<>();
    private volatile boolean running = false;
    private final Notifier _receiveNotifier;

    public VisionProcessor() {
        context = ZMQ.context(1);
        firstRunSetup(); 
        _receiveNotifier = new Notifier(this::receive); 
    }

    private void firstRunSetup() {
        Thread.currentThread().setPriority(2);
        Thread.currentThread().setName("Vision Thread");
    }

    public void createSubscriber(String topic, String addr) {
        ZMQ.Socket subscriber = context.socket(SocketType.SUB);
        subscriber.connect(addr);
        subscriber.subscribe(topic.getBytes());
        subscribers.put(topic, subscriber);
    }

    public synchronized void createPublisher(String topic, String addr) {
        ZMQ.Socket publisher = context.socket(SocketType.PUB);
        publisher.bind(addr); 
        publishers.put(topic, publisher);
    }

    public void start() {
        running = true;
        _receiveNotifier.startPeriodic(0.02);
    }

    public void stop() {
        running = false;
        _receiveNotifier.stop();
    }

    private void receive() {
        subscribers.values().forEach(this::processSubscriber);
    }


    private void processSubscriber(ZMQ.Socket subscriber) {
        String topic = subscriber.recvStr();
        if (topic != null) {
            byte[] data = subscriber.recv();
            if (data != null) {
                switch (topic) {
                    case "HeartBeat":
                        //HeartBeat heartBeat = decodeHeartBeat(data);
                        // Process HeartBeat data
                        break;
                    case "IMUInfo":
                        //IMUInfo imuInfo = decodeIMUInfo(data);
                        // Process IMUInfo data
                        break;
                    case "VisionPose":
                        //VisionPose visionPose = decodeVisionPose(data);
                        // Process VisionPose data
                        break;
                    default:
                        System.out.println("Unknown topic: " + topic);
                        break;
                }
            }
        }
    }
        
    public synchronized void sendData(String topic, byte[] data) {
        ZMQ.Socket publisher = publishers.get(topic);
        if (publisher != null) {
            publisher.sendMore(topic);
            publisher.send(data);
        }
    }
}
