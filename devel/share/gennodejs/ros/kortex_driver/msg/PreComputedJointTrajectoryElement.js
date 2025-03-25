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

class PreComputedJointTrajectoryElement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angles = null;
      this.joint_speeds = null;
      this.joint_accelerations = null;
      this.time_from_start = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angles')) {
        this.joint_angles = initObj.joint_angles
      }
      else {
        this.joint_angles = [];
      }
      if (initObj.hasOwnProperty('joint_speeds')) {
        this.joint_speeds = initObj.joint_speeds
      }
      else {
        this.joint_speeds = [];
      }
      if (initObj.hasOwnProperty('joint_accelerations')) {
        this.joint_accelerations = initObj.joint_accelerations
      }
      else {
        this.joint_accelerations = [];
      }
      if (initObj.hasOwnProperty('time_from_start')) {
        this.time_from_start = initObj.time_from_start
      }
      else {
        this.time_from_start = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PreComputedJointTrajectoryElement
    // Serialize message field [joint_angles]
    bufferOffset = _arraySerializer.float32(obj.joint_angles, buffer, bufferOffset, null);
    // Serialize message field [joint_speeds]
    bufferOffset = _arraySerializer.float32(obj.joint_speeds, buffer, bufferOffset, null);
    // Serialize message field [joint_accelerations]
    bufferOffset = _arraySerializer.float32(obj.joint_accelerations, buffer, bufferOffset, null);
    // Serialize message field [time_from_start]
    bufferOffset = _serializer.float32(obj.time_from_start, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PreComputedJointTrajectoryElement
    let len;
    let data = new PreComputedJointTrajectoryElement(null);
    // Deserialize message field [joint_angles]
    data.joint_angles = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [joint_speeds]
    data.joint_speeds = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [joint_accelerations]
    data.joint_accelerations = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [time_from_start]
    data.time_from_start = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.joint_angles.length;
    length += 4 * object.joint_speeds.length;
    length += 4 * object.joint_accelerations.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/PreComputedJointTrajectoryElement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da991996b59092cd15d7abc565fb075e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new PreComputedJointTrajectoryElement(null);
    if (msg.joint_angles !== undefined) {
      resolved.joint_angles = msg.joint_angles;
    }
    else {
      resolved.joint_angles = []
    }

    if (msg.joint_speeds !== undefined) {
      resolved.joint_speeds = msg.joint_speeds;
    }
    else {
      resolved.joint_speeds = []
    }

    if (msg.joint_accelerations !== undefined) {
      resolved.joint_accelerations = msg.joint_accelerations;
    }
    else {
      resolved.joint_accelerations = []
    }

    if (msg.time_from_start !== undefined) {
      resolved.time_from_start = msg.time_from_start;
    }
    else {
      resolved.time_from_start = 0.0
    }

    return resolved;
    }
};

module.exports = PreComputedJointTrajectoryElement;
