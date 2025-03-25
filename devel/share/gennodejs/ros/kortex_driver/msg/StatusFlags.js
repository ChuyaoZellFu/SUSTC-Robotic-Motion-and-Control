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

class StatusFlags {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StatusFlags
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StatusFlags
    let len;
    let data = new StatusFlags(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/StatusFlags';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94f8e368a8b96139de6828bbd6ed2797';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNKNOWN_STATUS = 0
    
    uint32 STABILIZED = 16
    
    uint32 MOTOR_INDEXED = 32
    
    uint32 MOTOR_INDEXING = 64
    
    uint32 JOINT_INDEXED = 128
    
    uint32 JOINT_INDEXING = 256
    
    uint32 HIGH_PRECISION = 512
    
    uint32 BRAKING = 1024
    
    uint32 SERVOING = 2048
    
    uint32 MAJOR_FAULT = 4096
    
    uint32 MINOR_FAULT = 8192
    
    uint32 CALIBRATED_TORQUE = 16384
    
    uint32 CALIBRATED_MAG_SENSOR = 32768
    
    uint32 CALIBRATED_ZERO = 65536
    
    uint32 GPIO_0 = 131072
    
    uint32 GPIO_1 = 262144
    
    uint32 CS_QUASI_STATIC_CONTACT = 524288
    
    uint32 CS_TRANSIENT_CONTACT = 1048576
    
    uint32 VFD_HALL_SYNC = 2097152
    
    uint32 VFD_INDEXED = 4194304
    
    uint32 DRIVE_BOARD_READY = 8388608
    
    uint32 CALIBRATED_CURRENT = 16777216
    
    uint32 CALIBRATED_MOTOR = 33554432
    
    uint32 SW0_ACTIVE = 67108864
    
    uint32 SW1_ACTIVE = 134217728
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StatusFlags(null);
    return resolved;
    }
};

// Constants for message
StatusFlags.Constants = {
  UNKNOWN_STATUS: 0,
  STABILIZED: 16,
  MOTOR_INDEXED: 32,
  MOTOR_INDEXING: 64,
  JOINT_INDEXED: 128,
  JOINT_INDEXING: 256,
  HIGH_PRECISION: 512,
  BRAKING: 1024,
  SERVOING: 2048,
  MAJOR_FAULT: 4096,
  MINOR_FAULT: 8192,
  CALIBRATED_TORQUE: 16384,
  CALIBRATED_MAG_SENSOR: 32768,
  CALIBRATED_ZERO: 65536,
  GPIO_0: 131072,
  GPIO_1: 262144,
  CS_QUASI_STATIC_CONTACT: 524288,
  CS_TRANSIENT_CONTACT: 1048576,
  VFD_HALL_SYNC: 2097152,
  VFD_INDEXED: 4194304,
  DRIVE_BOARD_READY: 8388608,
  CALIBRATED_CURRENT: 16777216,
  CALIBRATED_MOTOR: 33554432,
  SW0_ACTIVE: 67108864,
  SW1_ACTIVE: 134217728,
}

module.exports = StatusFlags;
