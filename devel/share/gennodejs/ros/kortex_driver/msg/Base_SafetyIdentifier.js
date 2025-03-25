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

class Base_SafetyIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Base_SafetyIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Base_SafetyIdentifier
    let len;
    let data = new Base_SafetyIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Base_SafetyIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1af24619c0f6e3d61a2eb98a96e27e96';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_BASE_SAFETY_IDENTIFIER = 0
    
    uint32 FIRMWARE_UPDATE_FAILURE = 1
    
    uint32 EXTERNAL_COMMUNICATION_ERROR = 2
    
    uint32 MAXIMUM_AMBIENT_TEMPERATURE = 4
    
    uint32 MAXIMUM_CORE_TEMPERATURE = 8
    
    uint32 JOINT_FAULT = 16
    
    uint32 CYCLIC_DATA_JITTER = 32
    
    uint32 REACHED_MAXIMUM_EVENT_LOGS = 64
    
    uint32 NO_KINEMATICS_SUPPORT = 128
    
    uint32 BRAKE_REMOVAL_FAILURE = 256
    
    uint32 NETWORK_ERROR = 512
    
    uint32 UNABLE_TO_REACH_POSE = 1024
    
    uint32 JOINT_DETECTION_ERROR = 2048
    
    uint32 NETWORK_INITIALIZATION_ERROR = 4096
    
    uint32 MAXIMUM_CURRENT = 8192
    
    uint32 MAXIMUM_VOLTAGE = 16384
    
    uint32 MINIMUM_VOLTAGE = 32768
    
    uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY = 65536
    
    uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY = 131072
    
    uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION = 262144
    
    uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION = 524288
    
    uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE = 1048576
    
    uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE = 2097152
    
    uint32 MAXIMUM_END_EFFECTOR_PAYLOAD = 4194304
    
    uint32 EMERGENCY_STOP_ACTIVATED = 8388608
    
    uint32 EMERGENCY_LINE_ACTIVATED = 16777216
    
    uint32 INRUSH_CURRENT_LIMITER_FAULT = 33554432
    
    uint32 NVRAM_CORRUPTED = 67108864
    
    uint32 INCOMPATIBLE_FIRMWARE_VERSION = 134217728
    
    uint32 POWERON_SELF_TEST_FAILURE = 268435456
    
    uint32 DISCRETE_INPUT_STUCK_ACTIVE = 536870912
    
    uint32 ARM_INTO_ILLEGAL_POSITION = 1073741824
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Base_SafetyIdentifier(null);
    return resolved;
    }
};

// Constants for message
Base_SafetyIdentifier.Constants = {
  UNSPECIFIED_BASE_SAFETY_IDENTIFIER: 0,
  FIRMWARE_UPDATE_FAILURE: 1,
  EXTERNAL_COMMUNICATION_ERROR: 2,
  MAXIMUM_AMBIENT_TEMPERATURE: 4,
  MAXIMUM_CORE_TEMPERATURE: 8,
  JOINT_FAULT: 16,
  CYCLIC_DATA_JITTER: 32,
  REACHED_MAXIMUM_EVENT_LOGS: 64,
  NO_KINEMATICS_SUPPORT: 128,
  BRAKE_REMOVAL_FAILURE: 256,
  NETWORK_ERROR: 512,
  UNABLE_TO_REACH_POSE: 1024,
  JOINT_DETECTION_ERROR: 2048,
  NETWORK_INITIALIZATION_ERROR: 4096,
  MAXIMUM_CURRENT: 8192,
  MAXIMUM_VOLTAGE: 16384,
  MINIMUM_VOLTAGE: 32768,
  MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY: 65536,
  MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY: 131072,
  MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION: 262144,
  MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION: 524288,
  MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE: 1048576,
  MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE: 2097152,
  MAXIMUM_END_EFFECTOR_PAYLOAD: 4194304,
  EMERGENCY_STOP_ACTIVATED: 8388608,
  EMERGENCY_LINE_ACTIVATED: 16777216,
  INRUSH_CURRENT_LIMITER_FAULT: 33554432,
  NVRAM_CORRUPTED: 67108864,
  INCOMPATIBLE_FIRMWARE_VERSION: 134217728,
  POWERON_SELF_TEST_FAILURE: 268435456,
  DISCRETE_INPUT_STUCK_ACTIVE: 536870912,
  ARM_INTO_ILLEGAL_POSITION: 1073741824,
}

module.exports = Base_SafetyIdentifier;
