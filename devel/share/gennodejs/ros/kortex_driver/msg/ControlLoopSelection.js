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

class ControlLoopSelection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlLoopSelection
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlLoopSelection
    let len;
    let data = new ControlLoopSelection(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControlLoopSelection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85d717df01106b4f27c7c235a5980355';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 RESERVED = 0
    
    uint32 JOINT_POSITION = 1
    
    uint32 MOTOR_POSITION = 2
    
    uint32 JOINT_VELOCITY = 4
    
    uint32 MOTOR_VELOCITY = 8
    
    uint32 JOINT_TORQUE = 16
    
    uint32 MOTOR_CURRENT = 32
    
    uint32 JOINT_TORQUE_HIGH_VELOCITY = 64
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlLoopSelection(null);
    return resolved;
    }
};

// Constants for message
ControlLoopSelection.Constants = {
  RESERVED: 0,
  JOINT_POSITION: 1,
  MOTOR_POSITION: 2,
  JOINT_VELOCITY: 4,
  MOTOR_VELOCITY: 8,
  JOINT_TORQUE: 16,
  MOTOR_CURRENT: 32,
  JOINT_TORQUE_HIGH_VELOCITY: 64,
}

module.exports = ControlLoopSelection;
