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

class TwistAngularSoftLimit {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_mode = null;
      this.twist_angular_soft_limit = null;
    }
    else {
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
      if (initObj.hasOwnProperty('twist_angular_soft_limit')) {
        this.twist_angular_soft_limit = initObj.twist_angular_soft_limit
      }
      else {
        this.twist_angular_soft_limit = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TwistAngularSoftLimit
    // Serialize message field [control_mode]
    bufferOffset = _serializer.uint32(obj.control_mode, buffer, bufferOffset);
    // Serialize message field [twist_angular_soft_limit]
    bufferOffset = _serializer.float32(obj.twist_angular_soft_limit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TwistAngularSoftLimit
    let len;
    let data = new TwistAngularSoftLimit(null);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [twist_angular_soft_limit]
    data.twist_angular_soft_limit = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TwistAngularSoftLimit';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec20d4ed4983e99d6172917731a2eee5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 control_mode
    float32 twist_angular_soft_limit
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TwistAngularSoftLimit(null);
    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    if (msg.twist_angular_soft_limit !== undefined) {
      resolved.twist_angular_soft_limit = msg.twist_angular_soft_limit;
    }
    else {
      resolved.twist_angular_soft_limit = 0.0
    }

    return resolved;
    }
};

module.exports = TwistAngularSoftLimit;
