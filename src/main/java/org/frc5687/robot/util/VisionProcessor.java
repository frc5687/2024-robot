package org.frc5687.robot.util;

import org.zeromq.SocketType;
import org.zeromq.ZMQ;
import org.zeromq.ZMQException;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

import org.frc5687.Messages.*;
import java.nio.ByteOrder;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class VisionProcessor {
    private final ZMQ.Context context;
    private final Map<String, ZMQ.Socket> subscribers = new HashMap<>();
    private final Map<String, ZMQ.Socket> publishers = new HashMap<>();
    private volatile boolean running = false;
    private boolean firstRun = true;
    private ReentrantLock _mtx;

    private VisionPoseArray _detectedObjects = new VisionPoseArray();

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

    public void createSubscriber(String subscriberName, String... topics) {
        ZMQ.Socket subscriber = context.socket(SocketType.SUB);
        for (String topic : topics) {
            subscriber.subscribe(topic.getBytes());
        }
        subscriber.setReceiveTimeOut(1000);
        subscribers.put(subscriberName, subscriber);
    }

    public void connectSubscriber(String subscriberName, String addr) {
        ZMQ.Socket subscriber = subscribers.get(subscriberName);
        if (subscriber != null) {
            subscriber.connect(addr);
        }
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

    public VisionPoseArray getDetectedObjects() {
        try {
            _mtx.lock();
            return _detectedObjects;
        } finally {
            _mtx.unlock();
        }
    }

    public void processSubscriber(ZMQ.Socket subscriber) {
        try {
            String topic = subscriber.recvStr();
            // DriverStation.reportError("Topic found: "+ topic, false);
            if (topic != null) {
                byte[] data = subscriber.recv();
                if (data != null) {
                    switch (topic) {
                        case "Objects":
                            try {
                                _mtx.lock();
                                ByteBuffer bb = ByteBuffer.wrap(data);
                                bb.order(ByteOrder.LITTLE_ENDIAN);
                                _detectedObjects = VisionPoseArray.getRootAsVisionPoseArray(bb);
                            } finally {
                                _mtx.unlock();
                            }
                            break;
                    }
                }
            }
        } catch (ZMQException e) {
            if (e.getErrorCode() == ZMQ.Error.EAGAIN.getCode()) {
                // No message received within the timeout period
                // DriverStation.reportError("No AprilTags message received within the timeout period", false);
            } else {
                // Handle other ZMQ exceptions
                e.printStackTrace();
            }
        }
    }
}