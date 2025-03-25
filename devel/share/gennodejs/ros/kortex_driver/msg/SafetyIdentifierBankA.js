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

class SafetyIdentifierBankA {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyIdentifierBankA
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyIdentifierBankA
    let len;
    let data = new SafetyIdentifierBankA(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyIdentifierBankA';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e8cfb814cd11bbacd2f40df395053aa0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER_BANK_A = 0
    
    uint32 FOLLOWING_ERROR = 1
    
    uint32 MAXIMUM_VELOCITY = 2
    
    uint32 JOINT_LIMIT_HIGH = 4
    
    uint32 JOINT_LIMIT_LOW = 8
    
    uint32 STRAIN_GAUGE_MISMATCH = 16
    
    uint32 MAXIMUM_TORQUE = 32
    
    uint32 UNRELIABLE_ABSOLUTE_POSITION = 64
    
    uint32 MAGNETIC_POSITION = 128
    
    uint32 HALL_POSITION = 256
    
    uint32 HALL_SEQUENCE = 512
    
    uint32 INPUT_ENCODER_HALL_MISMATCH = 1024
    
    uint32 INPUT_ENCODER_INDEX_MISMATCH = 2048
    
    uint32 INPUT_ENCODER_MAGNETIC_MISMATCH = 4096
    
    uint32 MAXIMUM_MOTOR_CURRENT = 8192
    
    uint32 MOTOR_CURRENT_MISMATCH = 16384
    
    uint32 MAXIMUM_VOLTAGE = 32768
    
    uint32 MINIMUM_VOLTAGE = 65536
    
    uint32 MAXIMUM_MOTOR_TEMPERATURE = 131072
    
    uint32 MAXIMUM_CORE_TEMPERATURE = 262144
    
    uint32 NON_VOLATILE_MEMORY_CORRUPTED = 524288
    
    uint32 MOTOR_DRIVER_FAULT = 1048576
    
    uint32 EMERGENCY_LINE_ASSERTED = 2097152
    
    uint32 COMMUNICATION_TICK_LOST = 4194304
    
    uint32 WATCHDOG_TRIGGERED = 8388608
    
    uint32 UNRELIABLE_CAPACITIVE_SENSOR = 16777216
    
    uint32 UNEXPECTED_GEAR_RATIO = 33554432
    
    uint32 HALL_MAGNETIC_MISMATCH = 67108864
    
    uint32 BRAKE_DRIVE_FAULT = 134217728
    
    uint32 BRAKE_RELEASE_MOTION_OUT_OF_RANGE = 268435456
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SafetyIdentifierBankA(null);
    return resolved;
    }
};

// Constants for message
SafetyIdentifierBankA.Constants = {
  UNSPECIFIED_ACTUATOR_SAFETY_IDENTIFIER_BANK_A: 0,
  FOLLOWING_ERROR: 1,
  MAXIMUM_VELOCITY: 2,
  JOINT_LIMIT_HIGH: 4,
  JOINT_LIMIT_LOW: 8,
  STRAIN_GAUGE_MISMATCH: 16,
  MAXIMUM_TORQUE: 32,
  UNRELIABLE_ABSOLUTE_POSITION: 64,
  MAGNETIC_POSITION: 128,
  HALL_POSITION: 256,
  HALL_SEQUENCE: 512,
  INPUT_ENCODER_HALL_MISMATCH: 1024,
  INPUT_ENCODER_INDEX_MISMATCH: 2048,
  INPUT_ENCODER_MAGNETIC_MISMATCH: 4096,
  MAXIMUM_MOTOR_CURRENT: 8192,
  MOTOR_CURRENT_MISMATCH: 16384,
  MAXIMUM_VOLTAGE: 32768,
  MINIMUM_VOLTAGE: 65536,
  MAXIMUM_MOTOR_TEMPERATURE: 131072,
  MAXIMUM_CORE_TEMPERATURE: 262144,
  NON_VOLATILE_MEMORY_CORRUPTED: 524288,
  MOTOR_DRIVER_FAULT: 1048576,
  EMERGENCY_LINE_ASSERTED: 2097152,
  COMMUNICATION_TICK_LOST: 4194304,
  WATCHDOG_TRIGGERED: 8388608,
  UNRELIABLE_CAPACITIVE_SENSOR: 16777216,
  UNEXPECTED_GEAR_RATIO: 33554432,
  HALL_MAGNETIC_MISMATCH: 67108864,
  BRAKE_DRIVE_FAULT: 134217728,
  BRAKE_RELEASE_MOTION_OUT_OF_RANGE: 268435456,
}

module.exports = SafetyIdentifierBankA;
