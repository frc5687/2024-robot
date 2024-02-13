package org.frc5687.robot.util;

import org.zeromq.SocketType;
import org.zeromq.ZMQ;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

import org.frc5687.Messages.VisionPose;
import org.frc5687.Messages.VisionPoseArray;
import java.nio.ByteOrder;
import java.nio.ByteBuffer;

import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.wpilibj.Notifier;

public class VisionProcessor {
    private final ZMQ.Context context;
    private final Map<String, ZMQ.Socket> subscribers = new HashMap<>();
    private final Map<String, ZMQ.Socket> publishers = new HashMap<>();
    private volatile boolean running = false;
    private boolean firstRun = true;
    private ReentrantLock _mtx;

    private VisionPoseArray _detectedObjects;
    private final Notifier _receiveNotifier =
            new Notifier(
                    () -> {
                        synchronized (VisionProcessor.this) {
                            if (firstRun) {
                                firstRunSetup();
                                firstRun = false;
                            }
                            receive();
                        }
                    });

    public VisionProcessor() {
        context = ZMQ.context(1);
        _mtx = new ReentrantLock();
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

    public ZMQ.Socket getSubscriber(String topic) {
        return subscribers.get(topic);
    }

    private void receive() {
        subscribers.values().forEach(this::processSubscriber);
    }

    public VisionPoseArray getDetectedObjects() {
        try {
            _mtx.lock();
        return _detectedObjects;
        } finally {
            _mtx.unlock();
        }
    }

    public void processSubscriber(ZMQ.Socket subscriber) {
        String topic = subscriber.recvStr();
        if (topic != null) {
            byte[] data = subscriber.recv();
            if (data != null) {
                switch (topic) {
                    case "HeartBeat":
                        System.out.println("Not implemented HeartBeat");
                        // Assuming HeartBeat decoding is implemented elsewhere
                        break;
                    case "IMUInfo":
                        // Assuming IMUInfo decoding is implemented elsewhere
                        break;
                    case "VisionPose":
                        ByteBuffer bb1 = ByteBuffer.wrap(data);
                        VisionPose visionPose1 = VisionPose.getRootAsVisionPose(bb1);
                        System.out.println("VisionPose{ x: " + visionPose1.x() + " y: " + visionPose1.y() + " z: " + visionPose1.z() + " }");
                        break;
                    case "Objects":
                        try {
                            _mtx.lock();
                            ByteBuffer bb = ByteBuffer.wrap(data);
                            bb.order(ByteOrder.LITTLE_ENDIAN);
                            _detectedObjects = VisionPoseArray.getRootAsVisionPoseArray(bb);
                            //for (int i = 0; i < visionPoseArray.posesLength(); i++) {
                            //    VisionPose visionPose = visionPoseArray.poses(i);
                            //    System.out.println("VisionPose " + i + "{ x: " + visionPose.x() + ", y: " + visionPose.y() + ", z: " + visionPose.z() + " }");
                            //}
                        } catch (IndexOutOfBoundsException e) {
                            System.err.println("Failed to process VisionPoseArray due to IndexOutOfBoundsException.");
                            // Log additional details here
                        } finally {
                            _mtx.unlock();

                        }              
                }
            }
        }
    }
}