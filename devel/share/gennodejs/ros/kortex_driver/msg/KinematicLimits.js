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

class KinematicLimits {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_mode = null;
      this.twist_linear = null;
      this.twist_angular = null;
      this.joint_speed_limits = null;
      this.joint_acceleration_limits = null;
    }
    else {
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = 0;
      }
      if (initObj.hasOwnProperty('twist_linear')) {
        this.twist_linear = initObj.twist_linear
      }
      else {
        this.twist_linear = 0.0;
      }
      if (initObj.hasOwnProperty('twist_angular')) {
        this.twist_angular = initObj.twist_angular
      }
      else {
        this.twist_angular = 0.0;
      }
      if (initObj.hasOwnProperty('joint_speed_limits')) {
        this.joint_speed_limits = initObj.joint_speed_limits
      }
      else {
        this.joint_speed_limits = [];
      }
      if (initObj.hasOwnProperty('joint_acceleration_limits')) {
        this.joint_acceleration_limits = initObj.joint_acceleration_limits
      }
      else {
        this.joint_acceleration_limits = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KinematicLimits
    // Serialize message field [control_mode]
    bufferOffset = _serializer.uint32(obj.control_mode, buffer, bufferOffset);
    // Serialize message field [twist_linear]
    bufferOffset = _serializer.float32(obj.twist_linear, buffer, bufferOffset);
    // Serialize message field [twist_angular]
    bufferOffset = _serializer.float32(obj.twist_angular, buffer, bufferOffset);
    // Serialize message field [joint_speed_limits]
    bufferOffset = _arraySerializer.float32(obj.joint_speed_limits, buffer, bufferOffset, null);
    // Serialize message field [joint_acceleration_limits]
    bufferOffset = _arraySerializer.float32(obj.joint_acceleration_limits, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KinematicLimits
    let len;
    let data = new KinematicLimits(null);
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [twist_linear]
    data.twist_linear = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [twist_angular]
    data.twist_angular = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint_speed_limits]
    data.joint_speed_limits = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [joint_acceleration_limits]
    data.joint_acceleration_limits = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.joint_speed_limits.length;
    length += 4 * object.joint_acceleration_limits.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/KinematicLimits';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5496cda94486ec0fe4a0697dda6200aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 control_mode
    float32 twist_linear
    float32 twist_angular
    float32[] joint_speed_limits
    float32[] joint_acceleration_limits
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KinematicLimits(null);
    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = 0
    }

    if (msg.twist_linear !== undefined) {
      resolved.twist_linear = msg.twist_linear;
    }
    else {
      resolved.twist_linear = 0.0
    }

    if (msg.twist_angular !== undefined) {
      resolved.twist_angular = msg.twist_angular;
    }
    else {
      resolved.twist_angular = 0.0
    }

    if (msg.joint_speed_limits !== undefined) {
      resolved.joint_speed_limits = msg.joint_speed_limits;
    }
    else {
      resolved.joint_speed_limits = []
    }

    if (msg.joint_acceleration_limits !== undefined) {
      resolved.joint_acceleration_limits = msg.joint_acceleration_limits;
    }
    else {
      resolved.joint_acceleration_limits = []
    }

    return resolved;
    }
};

module.exports = KinematicLimits;
