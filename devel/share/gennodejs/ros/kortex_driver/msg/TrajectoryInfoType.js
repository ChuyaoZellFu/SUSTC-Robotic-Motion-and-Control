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

class TrajectoryInfoType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryInfoType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryInfoType
    let len;
    let data = new TrajectoryInfoType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TrajectoryInfoType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8bf652b45448cd88f4d8d2fc90a3634d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_TRAJECTORY_INFORMATION = 0
    
    uint32 JOINT_ACCELERATION_LIMIT_REACHED = 1
    
    uint32 JOINT_SPEED_LIMIT_REACHED = 2
    
    uint32 JOINT_POSITION_LIMIT_REACHED = 3
    
    uint32 JOINT_TORQUE_LIMIT_REACHED = 4
    
    uint32 SINGULARITY_REGION = 5
    
    uint32 INVERSE_KINEMATIC_FAILED = 6
    
    uint32 CARTESIAN_ACCELERATION_LIMIT_REACHED = 7
    
    uint32 CARTESIAN_SPEED_LIMIT_REACHED = 8
    
    uint32 CARTESIAN_POSITION_LIMIT_REACHED = 9
    
    uint32 CARTESIAN_WRENCH_LIMIT_REACHED = 10
    
    uint32 ENTERING_PROTECTION_ZONE = 11
    
    uint32 WAYPOINT_REACHED = 12
    
    uint32 TRAJECTORY_OK = 13
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryInfoType(null);
    return resolved;
    }
};

// Constants for message
TrajectoryInfoType.Constants = {
  UNSPECIFIED_TRAJECTORY_INFORMATION: 0,
  JOINT_ACCELERATION_LIMIT_REACHED: 1,
  JOINT_SPEED_LIMIT_REACHED: 2,
  JOINT_POSITION_LIMIT_REACHED: 3,
  JOINT_TORQUE_LIMIT_REACHED: 4,
  SINGULARITY_REGION: 5,
  INVERSE_KINEMATIC_FAILED: 6,
  CARTESIAN_ACCELERATION_LIMIT_REACHED: 7,
  CARTESIAN_SPEED_LIMIT_REACHED: 8,
  CARTESIAN_POSITION_LIMIT_REACHED: 9,
  CARTESIAN_WRENCH_LIMIT_REACHED: 10,
  ENTERING_PROTECTION_ZONE: 11,
  WAYPOINT_REACHED: 12,
  TRAJECTORY_OK: 13,
}

module.exports = TrajectoryInfoType;
