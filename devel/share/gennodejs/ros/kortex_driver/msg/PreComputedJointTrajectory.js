// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');

//-----------------------------------------------------------

class PreComputedJointTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.trajectory_elements = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('trajectory_elements')) {
        this.trajectory_elements = initObj.trajectory_elements
      }
      else {
        this.trajectory_elements = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PreComputedJointTrajectory
    // Serialize message field [mode]
    bufferOffset = _serializer.uint32(obj.mode, buffer, bufferOffset);
    // Serialize message field [trajectory_elements]
    // Serialize the length for message field [trajectory_elements]
    bufferOffset = _serializer.uint32(obj.trajectory_elements.length, buffer, bufferOffset);
    obj.trajectory_elements.forEach((val) => {
      bufferOffset = PreComputedJointTrajectoryElement.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PreComputedJointTrajectory
    let len;
    let data = new PreComputedJointTrajectory(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [trajectory_elements]
    // Deserialize array length for message field [trajectory_elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.trajectory_elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.trajectory_elements[i] = PreComputedJointTrajectoryElement.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.trajectory_elements.forEach((val) => {
      length += PreComputedJointTrajectoryElement.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/PreComputedJointTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0b9aae85f07814f6f429b1dde77280f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 mode
    PreComputedJointTrajectoryElement[] trajectory_elements
    ================================================================================
    MSG: kortex_driver/PreComputedJointTrajectoryElement
    
    float32[] joint_angles
    float32[] joint_speeds
    float32[] joint_accelerations
    float32 time_from_start
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PreComputedJointTrajectory(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.trajectory_elements !== undefined) {
      resolved.trajectory_elements = new Array(msg.trajectory_elements.length);
      for (let i = 0; i < resolved.trajectory_elements.length; ++i) {
        resolved.trajectory_elements[i] = PreComputedJointTrajectoryElement.Resolve(msg.trajectory_elements[i]);
      }
    }
    else {
      resolved.trajectory_elements = []
    }

    return resolved;
    }
};

module.exports = PreComputedJointTrajectory;
