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

class ActionType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionType
    let len;
    let data = new ActionType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActionType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2814fa62baddf71292e9d832616cc515';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_ACTION = 0
    
    uint32 SEND_TWIST_COMMAND = 1
    
    uint32 SEND_WRENCH_COMMAND = 2
    
    uint32 SEND_JOINT_SPEEDS = 4
    
    uint32 REACH_POSE = 6
    
    uint32 REACH_JOINT_ANGLES = 7
    
    uint32 TOGGLE_ADMITTANCE_MODE = 13
    
    uint32 SNAPSHOT = 14
    
    uint32 SWITCH_CONTROL_MAPPING = 16
    
    uint32 NAVIGATE_JOINTS = 17
    
    uint32 NAVIGATE_MAPPINGS = 18
    
    uint32 CHANGE_TWIST = 22
    
    uint32 CHANGE_JOINT_SPEEDS = 23
    
    uint32 CHANGE_WRENCH = 25
    
    uint32 APPLY_EMERGENCY_STOP = 28
    
    uint32 CLEAR_FAULTS = 29
    
    uint32 TIME_DELAY = 31
    
    uint32 EXECUTE_ACTION = 32
    
    uint32 SEND_GRIPPER_COMMAND = 33
    
    uint32 SEND_GPIO_COMMAND = 34
    
    uint32 STOP_ACTION = 35
    
    uint32 PLAY_PRE_COMPUTED_TRAJECTORY = 39
    
    uint32 EXECUTE_SEQUENCE = 40
    
    uint32 EXECUTE_WAYPOINT_LIST = 41
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionType(null);
    return resolved;
    }
};

// Constants for message
ActionType.Constants = {
  UNSPECIFIED_ACTION: 0,
  SEND_TWIST_COMMAND: 1,
  SEND_WRENCH_COMMAND: 2,
  SEND_JOINT_SPEEDS: 4,
  REACH_POSE: 6,
  REACH_JOINT_ANGLES: 7,
  TOGGLE_ADMITTANCE_MODE: 13,
  SNAPSHOT: 14,
  SWITCH_CONTROL_MAPPING: 16,
  NAVIGATE_JOINTS: 17,
  NAVIGATE_MAPPINGS: 18,
  CHANGE_TWIST: 22,
  CHANGE_JOINT_SPEEDS: 23,
  CHANGE_WRENCH: 25,
  APPLY_EMERGENCY_STOP: 28,
  CLEAR_FAULTS: 29,
  TIME_DELAY: 31,
  EXECUTE_ACTION: 32,
  SEND_GRIPPER_COMMAND: 33,
  SEND_GPIO_COMMAND: 34,
  STOP_ACTION: 35,
  PLAY_PRE_COMPUTED_TRAJECTORY: 39,
  EXECUTE_SEQUENCE: 40,
  EXECUTE_WAYPOINT_LIST: 41,
}

module.exports = ActionType;
