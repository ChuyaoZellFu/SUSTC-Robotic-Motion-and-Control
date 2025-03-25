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

class ControlLoopParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loop_selection = null;
      this.error_saturation = null;
      this.output_saturation = null;
      this.kAz = null;
      this.kBz = null;
      this.error_dead_band = null;
    }
    else {
      if (initObj.hasOwnProperty('loop_selection')) {
        this.loop_selection = initObj.loop_selection
      }
      else {
        this.loop_selection = 0;
      }
      if (initObj.hasOwnProperty('error_saturation')) {
        this.error_saturation = initObj.error_saturation
      }
      else {
        this.error_saturation = 0.0;
      }
      if (initObj.hasOwnProperty('output_saturation')) {
        this.output_saturation = initObj.output_saturation
      }
      else {
        this.output_saturation = 0.0;
      }
      if (initObj.hasOwnProperty('kAz')) {
        this.kAz = initObj.kAz
      }
      else {
        this.kAz = [];
      }
      if (initObj.hasOwnProperty('kBz')) {
        this.kBz = initObj.kBz
      }
      else {
        this.kBz = [];
      }
      if (initObj.hasOwnProperty('error_dead_band')) {
        this.error_dead_band = initObj.error_dead_band
      }
      else {
        this.error_dead_band = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlLoopParameters
    // Serialize message field [loop_selection]
    bufferOffset = _serializer.uint32(obj.loop_selection, buffer, bufferOffset);
    // Serialize message field [error_saturation]
    bufferOffset = _serializer.float32(obj.error_saturation, buffer, bufferOffset);
    // Serialize message field [output_saturation]
    bufferOffset = _serializer.float32(obj.output_saturation, buffer, bufferOffset);
    // Serialize message field [kAz]
    bufferOffset = _arraySerializer.float32(obj.kAz, buffer, bufferOffset, null);
    // Serialize message field [kBz]
    bufferOffset = _arraySerializer.float32(obj.kBz, buffer, bufferOffset, null);
    // Serialize message field [error_dead_band]
    bufferOffset = _serializer.float32(obj.error_dead_band, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlLoopParameters
    let len;
    let data = new ControlLoopParameters(null);
    // Deserialize message field [loop_selection]
    data.loop_selection = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [error_saturation]
    data.error_saturation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [output_saturation]
    data.output_saturation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kAz]
    data.kAz = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [kBz]
    data.kBz = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [error_dead_band]
    data.error_dead_band = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.kAz.length;
    length += 4 * object.kBz.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControlLoopParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '309e14eef078d5bea86d94317d3d0e04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 loop_selection
    float32 error_saturation
    float32 output_saturation
    float32[] kAz
    float32[] kBz
    float32 error_dead_band
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlLoopParameters(null);
    if (msg.loop_selection !== undefined) {
      resolved.loop_selection = msg.loop_selection;
    }
    else {
      resolved.loop_selection = 0
    }

    if (msg.error_saturation !== undefined) {
      resolved.error_saturation = msg.error_saturation;
    }
    else {
      resolved.error_saturation = 0.0
    }

    if (msg.output_saturation !== undefined) {
      resolved.output_saturation = msg.output_saturation;
    }
    else {
      resolved.output_saturation = 0.0
    }

    if (msg.kAz !== undefined) {
      resolved.kAz = msg.kAz;
    }
    else {
      resolved.kAz = []
    }

    if (msg.kBz !== undefined) {
      resolved.kBz = msg.kBz;
    }
    else {
      resolved.kBz = []
    }

    if (msg.error_dead_band !== undefined) {
      resolved.error_dead_band = msg.error_dead_band;
    }
    else {
      resolved.error_dead_band = 0.0
    }

    return resolved;
    }
};

module.exports = ControlLoopParameters;
