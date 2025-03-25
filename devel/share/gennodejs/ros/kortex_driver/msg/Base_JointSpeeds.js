// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointSpeed = require('./JointSpeed.js');

//-----------------------------------------------------------

class Base_JointSpeeds {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_speeds = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_speeds')) {
        this.joint_speeds = initObj.joint_speeds
      }
      else {
        this.joint_speeds = [];
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Base_JointSpeeds
    // Serialize message field [joint_speeds]
    // Serialize the length for message field [joint_speeds]
    bufferOffset = _serializer.uint32(obj.joint_speeds.length, buffer, bufferOffset);
    obj.joint_speeds.forEach((val) => {
      bufferOffset = JointSpeed.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [duration]
    bufferOffset = _serializer.uint32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Base_JointSpeeds
    let len;
    let data = new Base_JointSpeeds(null);
    // Deserialize message field [joint_speeds]
    // Deserialize array length for message field [joint_speeds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joint_speeds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joint_speeds[i] = JointSpeed.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [duration]
    data.duration = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.joint_speeds.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Base_JointSpeeds';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f323c64d027d947f420c021696580eb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    JointSpeed[] joint_speeds
    uint32 duration
    ================================================================================
    MSG: kortex_driver/JointSpeed
    
    uint32 joint_identifier
    float32 value
    uint32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Base_JointSpeeds(null);
    if (msg.joint_speeds !== undefined) {
      resolved.joint_speeds = new Array(msg.joint_speeds.length);
      for (let i = 0; i < resolved.joint_speeds.length; ++i) {
        resolved.joint_speeds[i] = JointSpeed.Resolve(msg.joint_speeds[i]);
      }
    }
    else {
      resolved.joint_speeds = []
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0
    }

    return resolved;
    }
};

module.exports = Base_JointSpeeds;
