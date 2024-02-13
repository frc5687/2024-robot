// automatically generated by the FlatBuffers compiler, do not modify

package org.frc5687.Messages;

import com.google.flatbuffers.BaseVector;
import com.google.flatbuffers.BooleanVector;
import com.google.flatbuffers.ByteVector;
import com.google.flatbuffers.Constants;
import com.google.flatbuffers.DoubleVector;
import com.google.flatbuffers.FlatBufferBuilder;
import com.google.flatbuffers.FloatVector;
import com.google.flatbuffers.IntVector;
import com.google.flatbuffers.LongVector;
import com.google.flatbuffers.ShortVector;
import com.google.flatbuffers.StringVector;
import com.google.flatbuffers.Struct;
import com.google.flatbuffers.Table;
import com.google.flatbuffers.UnionVector;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@SuppressWarnings("unused")
public final class HeartBeat extends Table {
  public static void ValidateVersion() { Constants.FLATBUFFERS_23_5_26(); }
  public static HeartBeat getRootAsHeartBeat(ByteBuffer _bb) { return getRootAsHeartBeat(_bb, new HeartBeat()); }
  public static HeartBeat getRootAsHeartBeat(ByteBuffer _bb, HeartBeat obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__assign(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public void __init(int _i, ByteBuffer _bb) { __reset(_i, _bb); }
  public HeartBeat __assign(int _i, ByteBuffer _bb) { __init(_i, _bb); return this; }

  public int id() { int o = __offset(4); return o != 0 ? bb.getInt(o + bb_pos) : 0; }
  public long timestamp() { int o = __offset(6); return o != 0 ? bb.getLong(o + bb_pos) : 0L; }

  public static int createHeartBeat(FlatBufferBuilder builder,
      int id,
      long timestamp) {
    builder.startTable(2);
    HeartBeat.addTimestamp(builder, timestamp);
    HeartBeat.addId(builder, id);
    return HeartBeat.endHeartBeat(builder);
  }

  public static void startHeartBeat(FlatBufferBuilder builder) { builder.startTable(2); }
  public static void addId(FlatBufferBuilder builder, int id) { builder.addInt(0, id, 0); }
  public static void addTimestamp(FlatBufferBuilder builder, long timestamp) { builder.addLong(1, timestamp, 0L); }
  public static int endHeartBeat(FlatBufferBuilder builder) {
    int o = builder.endTable();
    return o;
  }
  public static void finishHeartBeatBuffer(FlatBufferBuilder builder, int offset) { builder.finish(offset); }
  public static void finishSizePrefixedHeartBeatBuffer(FlatBufferBuilder builder, int offset) { builder.finishSizePrefixed(offset); }

  public static final class Vector extends BaseVector {
    public Vector __assign(int _vector, int _element_size, ByteBuffer _bb) { __reset(_vector, _element_size, _bb); return this; }

    public HeartBeat get(int j) { return get(new HeartBeat(), j); }
    public HeartBeat get(HeartBeat obj, int j) {  return obj.__assign(__indirect(__element(j), bb), bb); }
  }
}
