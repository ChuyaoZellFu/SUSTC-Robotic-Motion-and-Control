// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FrequencyResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loop_selection = null;
      this.min_frequency = null;
      this.max_frequency = null;
      this.amplitude = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('loop_selection')) {
        this.loop_selection = initObj.loop_selection
      }
      else {
        this.loop_selection = 0;
      }
      if (initObj.hasOwnProperty('min_frequency')) {
        this.min_frequency = initObj.min_frequency
      }
      else {
        this.min_frequency = 0.0;
      }
      if (initObj.hasOwnProperty('max_frequency')) {
        this.max_frequency = initObj.max_frequency
      }
      else {
        this.max_frequency = 0.0;
      }
      if (initObj.hasOwnProperty('amplitude')) {
        this.amplitude = initObj.amplitude
      }
      else {
        this.amplitude = 0.0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrequencyResponse
    // Serialize message field [loop_selection]
    bufferOffset = _serializer.uint32(obj.loop_selection, buffer, bufferOffset);
    // Serialize message field [min_frequency]
    bufferOffset = _serializer.float32(obj.min_frequency, buffer, bufferOffset);
    // Serialize message field [max_frequency]
    bufferOffset = _serializer.float32(obj.max_frequency, buffer, bufferOffset);
    // Serialize message field [amplitude]
    bufferOffset = _serializer.float32(obj.amplitude, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrequencyResponse
    let len;
    let data = new FrequencyResponse(null);
    // Deserialize message field [loop_selection]
    data.loop_selection = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [min_frequency]
    data.min_frequency = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_frequency]
    data.max_frequency = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [amplitude]
    data.amplitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FrequencyResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86ed7a41c264d9e5ff7a52aa97222d1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 loop_selection
    float32 min_frequency
    float32 max_frequency
    float32 amplitude
    float32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FrequencyResponse(null);
    if (msg.loop_selection !== undefined) {
      resolved.loop_selection = msg.loop_selection;
    }
    else {
      resolved.loop_selection = 0
    }

    if (msg.min_frequency !== undefined) {
      resolved.min_frequency = msg.min_frequency;
    }
    else {
      resolved.min_frequency = 0.0
    }

    if (msg.max_frequency !== undefined) {
      resolved.max_frequency = msg.max_frequency;
    }
    else {
      resolved.max_frequency = 0.0
    }

    if (msg.amplitude !== undefined) {
      resolved.amplitude = msg.amplitude;
    }
    else {
      resolved.amplitude = 0.0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    return resolved;
    }
};

module.exports = FrequencyResponse;
