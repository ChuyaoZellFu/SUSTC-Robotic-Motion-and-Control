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

class ControlConfigurationEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlConfigurationEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlConfigurationEvent
    let len;
    let data = new ControlConfigurationEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControlConfigurationEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5aa2a442ccb0c93e20aaef58c6149541';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CONTROL_CONFIGURATION_EVENT = 0
    
    uint32 ANGLE_UNIT_CHANGED = 1
    
    uint32 GRAVITY_VECTOR_CHANGED = 2
    
    uint32 JOINT_ADMITTANCE_CONFIGURATION_CHANGED = 4
    
    uint32 NULL_ADMITTANCE_CONFIGURATION_CHANGED = 5
    
    uint32 CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED = 6
    
    uint32 JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED = 7
    
    uint32 WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED = 8
    
    uint32 WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED = 9
    
    uint32 CONTROL_CONFIGURATION_FACTORY_RESTORED = 10
    
    uint32 TOOL_CONFIGURATION_CHANGED = 11
    
    uint32 PAYLOAD_CONFIGURATION_CHANGED = 12
    
    uint32 CARTESIAN_REFERENCE_CHANGED = 13
    
    uint32 CHANGE_CONTROL_MODE_FAILED = 14
    
    uint32 JOINT_SPEED_SOFT_LIMITS_CHANGED = 16
    
    uint32 TWIST_LINEAR_SOFT_LIMIT_CHANGED = 17
    
    uint32 TWIST_ANGULAR_SOFT_LIMIT_CHANGED = 18
    
    uint32 JOINT_ACCELERATION_SOFT_LIMITS_CHANGED = 19
    
    uint32 DESIRED_TWIST_LINEAR_SPEED_CHANGED = 20
    
    uint32 DESIRED_TWIST_ANGULAR_SPEED_CHANGED = 21
    
    uint32 DESIRED_JOINT_SPEED_CHANGED = 22
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlConfigurationEvent(null);
    return resolved;
    }
};

// Constants for message
ControlConfigurationEvent.Constants = {
  UNSPECIFIED_CONTROL_CONFIGURATION_EVENT: 0,
  ANGLE_UNIT_CHANGED: 1,
  GRAVITY_VECTOR_CHANGED: 2,
  JOINT_ADMITTANCE_CONFIGURATION_CHANGED: 4,
  NULL_ADMITTANCE_CONFIGURATION_CHANGED: 5,
  CARTESIAN_ADMITTANCE_CONFIGURATION_CHANGED: 6,
  JOINT_TORQUE_HYBRID_CONFIGURATION_CHANGED: 7,
  WRENCH_COMMAND_NORMAL_CONFIGURATION_CHANGED: 8,
  WRENCH_COMMAND_RESTRICTED_CONFIGURATION_CHANGED: 9,
  CONTROL_CONFIGURATION_FACTORY_RESTORED: 10,
  TOOL_CONFIGURATION_CHANGED: 11,
  PAYLOAD_CONFIGURATION_CHANGED: 12,
  CARTESIAN_REFERENCE_CHANGED: 13,
  CHANGE_CONTROL_MODE_FAILED: 14,
  JOINT_SPEED_SOFT_LIMITS_CHANGED: 16,
  TWIST_LINEAR_SOFT_LIMIT_CHANGED: 17,
  TWIST_ANGULAR_SOFT_LIMIT_CHANGED: 18,
  JOINT_ACCELERATION_SOFT_LIMITS_CHANGED: 19,
  DESIRED_TWIST_LINEAR_SPEED_CHANGED: 20,
  DESIRED_TWIST_ANGULAR_SPEED_CHANGED: 21,
  DESIRED_JOINT_SPEED_CHANGED: 22,
}

module.exports = ControlConfigurationEvent;
