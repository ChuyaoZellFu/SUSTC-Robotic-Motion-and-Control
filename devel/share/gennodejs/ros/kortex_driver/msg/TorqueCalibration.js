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

class TorqueCalibration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.global_gain = null;
      this.global_offset = null;
      this.gain = null;
      this.offset = null;
    }
    else {
      if (initObj.hasOwnProperty('global_gain')) {
        this.global_gain = initObj.global_gain
      }
      else {
        this.global_gain = 0.0;
      }
      if (initObj.hasOwnProperty('global_offset')) {
        this.global_offset = initObj.global_offset
      }
      else {
        this.global_offset = 0.0;
      }
      if (initObj.hasOwnProperty('gain')) {
        this.gain = initObj.gain
      }
      else {
        this.gain = [];
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TorqueCalibration
    // Serialize message field [global_gain]
    bufferOffset = _serializer.float32(obj.global_gain, buffer, bufferOffset);
    // Serialize message field [global_offset]
    bufferOffset = _serializer.float32(obj.global_offset, buffer, bufferOffset);
    // Serialize message field [gain]
    bufferOffset = _arraySerializer.float32(obj.gain, buffer, bufferOffset, null);
    // Serialize message field [offset]
    bufferOffset = _arraySerializer.float32(obj.offset, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TorqueCalibration
    let len;
    let data = new TorqueCalibration(null);
    // Deserialize message field [global_gain]
    data.global_gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [global_offset]
    data.global_offset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gain]
    data.gain = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [offset]
    data.offset = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.gain.length;
    length += 4 * object.offset.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TorqueCalibration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '313d945f71d43fe1540908829effe904';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 global_gain
    float32 global_offset
    float32[] gain
    float32[] offset
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TorqueCalibration(null);
    if (msg.global_gain !== undefined) {
      resolved.global_gain = msg.global_gain;
    }
    else {
      resolved.global_gain = 0.0
    }

    if (msg.global_offset !== undefined) {
      resolved.global_offset = msg.global_offset;
    }
    else {
      resolved.global_offset = 0.0
    }

    if (msg.gain !== undefined) {
      resolved.gain = msg.gain;
    }
    else {
      resolved.gain = []
    }

    if (msg.offset !== undefined) {
      resolved.offset = msg.offset;
    }
    else {
      resolved.offset = []
    }

    return resolved;
    }
};

module.exports = TorqueCalibration;
