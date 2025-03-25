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

class TorqueOffset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.torque_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('torque_offset')) {
        this.torque_offset = initObj.torque_offset
      }
      else {
        this.torque_offset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TorqueOffset
    // Serialize message field [torque_offset]
    bufferOffset = _serializer.float32(obj.torque_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TorqueOffset
    let len;
    let data = new TorqueOffset(null);
    // Deserialize message field [torque_offset]
    data.torque_offset = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TorqueOffset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ac8f77f2604cb21bcfb1cb83c3c05e7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 torque_offset
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TorqueOffset(null);
    if (msg.torque_offset !== undefined) {
      resolved.torque_offset = msg.torque_offset;
    }
    else {
      resolved.torque_offset = 0.0
    }

    return resolved;
    }
};

module.exports = TorqueOffset;
