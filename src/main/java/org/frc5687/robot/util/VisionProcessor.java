package org.frc5687.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.zeromq.*;
import org.frc5687.Messages.*;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

public class VisionProcessor extends SubsystemBase {
    private final ZContext context;
    private final ConcurrentHashMap<String, ZMQ.Socket> subscribers = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, ZMQ.Socket> publishers = new ConcurrentHashMap<>();
    private volatile VisionPoseArray _detectedObjects = new VisionPoseArray();
    
    private final ConcurrentLinkedQueue<ByteBuffer> byteBufferPool = new ConcurrentLinkedQueue<>();

    public VisionProcessor() {
        context = new ZContext();
    }

    public void createSubscriber(String subscriberName, String... topics) {
        ZMQ.Socket subscriber = context.createSocket(SocketType.SUB);
        for (String topic : topics) {
            subscriber.subscribe(topic);
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

    public void createPublisher(String topic, String addr) {
        ZMQ.Socket publisher = context.createSocket(SocketType.PUB);
        publisher.bind(addr);
        publishers.put(topic, publisher);
    }

    @Override
    public void periodic() {
        for (ZMQ.Socket subscriber : subscribers.values()) {
            String topic = subscriber.recvStr();
            if (topic != null) {
                byte[] data = subscriber.recv();
                if (data != null && "Objects".equals(topic)) {
                    ByteBuffer bb = getByteBuffer(data.length);
                    bb.put(data);
                    bb.flip();
                    _detectedObjects = VisionPoseArray.getRootAsVisionPoseArray(bb);
                    releaseByteBuffer(bb);
                }
            }
        }
    }

    public VisionPoseArray getDetectedObjects() {
        return _detectedObjects;
    }

    public void close() {
        subscribers.values().forEach(ZMQ.Socket::close);
        publishers.values().forEach(ZMQ.Socket::close);
        context.close();
    }
    
    private ByteBuffer getByteBuffer(int capacity) {
        ByteBuffer bb = byteBufferPool.poll();
        if (bb == null || bb.capacity() < capacity) {
            bb = ByteBuffer.allocateDirect(capacity).order(ByteOrder.LITTLE_ENDIAN);
        } else {
            bb.clear();
        }
        return bb;
    }
    
    private void releaseByteBuffer(ByteBuffer bb) {
        byteBufferPool.offer(bb);
    }
}