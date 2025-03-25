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

class TrajectoryInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory_info_type = null;
      this.waypoint_index = null;
      this.joint_index = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory_info_type')) {
        this.trajectory_info_type = initObj.trajectory_info_type
      }
      else {
        this.trajectory_info_type = 0;
      }
      if (initObj.hasOwnProperty('waypoint_index')) {
        this.waypoint_index = initObj.waypoint_index
      }
      else {
        this.waypoint_index = 0;
      }
      if (initObj.hasOwnProperty('joint_index')) {
        this.joint_index = initObj.joint_index
      }
      else {
        this.joint_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryInfo
    // Serialize message field [trajectory_info_type]
    bufferOffset = _serializer.uint32(obj.trajectory_info_type, buffer, bufferOffset);
    // Serialize message field [waypoint_index]
    bufferOffset = _serializer.uint32(obj.waypoint_index, buffer, bufferOffset);
    // Serialize message field [joint_index]
    bufferOffset = _serializer.uint32(obj.joint_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryInfo
    let len;
    let data = new TrajectoryInfo(null);
    // Deserialize message field [trajectory_info_type]
    data.trajectory_info_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [waypoint_index]
    data.waypoint_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [joint_index]
    data.joint_index = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TrajectoryInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0eff35f5790d1aa2c620bfca263340d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 trajectory_info_type
    uint32 waypoint_index
    uint32 joint_index
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryInfo(null);
    if (msg.trajectory_info_type !== undefined) {
      resolved.trajectory_info_type = msg.trajectory_info_type;
    }
    else {
      resolved.trajectory_info_type = 0
    }

    if (msg.waypoint_index !== undefined) {
      resolved.waypoint_index = msg.waypoint_index;
    }
    else {
      resolved.waypoint_index = 0
    }

    if (msg.joint_index !== undefined) {
      resolved.joint_index = msg.joint_index;
    }
    else {
      resolved.joint_index = 0
    }

    return resolved;
    }
};

module.exports = TrajectoryInfo;
