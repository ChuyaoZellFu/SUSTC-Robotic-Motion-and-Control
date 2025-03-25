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

class EncoderDerivativeParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.max_window_width = null;
      this.min_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('max_window_width')) {
        this.max_window_width = initObj.max_window_width
      }
      else {
        this.max_window_width = 0;
      }
      if (initObj.hasOwnProperty('min_angle')) {
        this.min_angle = initObj.min_angle
      }
      else {
        this.min_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EncoderDerivativeParameters
    // Serialize message field [max_window_width]
    bufferOffset = _serializer.uint32(obj.max_window_width, buffer, bufferOffset);
    // Serialize message field [min_angle]
    bufferOffset = _serializer.float32(obj.min_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EncoderDerivativeParameters
    let len;
    let data = new EncoderDerivativeParameters(null);
    // Deserialize message field [max_window_width]
    data.max_window_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [min_angle]
    data.min_angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/EncoderDerivativeParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e0a5e3a584720a2550104b5e1faa434';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 max_window_width
    float32 min_angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EncoderDerivativeParameters(null);
    if (msg.max_window_width !== undefined) {
      resolved.max_window_width = msg.max_window_width;
    }
    else {
      resolved.max_window_width = 0
    }

    if (msg.min_angle !== undefined) {
      resolved.min_angle = msg.min_angle;
    }
    else {
      resolved.min_angle = 0.0
    }

    return resolved;
    }
};

module.exports = EncoderDerivativeParameters;
