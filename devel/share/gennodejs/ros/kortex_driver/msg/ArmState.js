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

class ArmState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmState
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmState
    let len;
    let data = new ArmState(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ArmState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac681740837f166518d3ae6a4e6eeda1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ARMSTATE_UNSPECIFIED = 0
    
    uint32 ARMSTATE_BASE_INITIALIZATION = 1
    
    uint32 ARMSTATE_IDLE = 2
    
    uint32 ARMSTATE_INITIALIZATION = 3
    
    uint32 ARMSTATE_IN_FAULT = 4
    
    uint32 ARMSTATE_MAINTENANCE = 5
    
    uint32 ARMSTATE_SERVOING_LOW_LEVEL = 6
    
    uint32 ARMSTATE_SERVOING_READY = 7
    
    uint32 ARMSTATE_SERVOING_PLAYING_SEQUENCE = 8
    
    uint32 ARMSTATE_SERVOING_MANUALLY_CONTROLLED = 9
    
    uint32 ARMSTATE_BRAKE_RELEASING = 10
    
    uint32 ARMSTATE_RESERVED = 255
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmState(null);
    return resolved;
    }
};

// Constants for message
ArmState.Constants = {
  ARMSTATE_UNSPECIFIED: 0,
  ARMSTATE_BASE_INITIALIZATION: 1,
  ARMSTATE_IDLE: 2,
  ARMSTATE_INITIALIZATION: 3,
  ARMSTATE_IN_FAULT: 4,
  ARMSTATE_MAINTENANCE: 5,
  ARMSTATE_SERVOING_LOW_LEVEL: 6,
  ARMSTATE_SERVOING_READY: 7,
  ARMSTATE_SERVOING_PLAYING_SEQUENCE: 8,
  ARMSTATE_SERVOING_MANUALLY_CONTROLLED: 9,
  ARMSTATE_BRAKE_RELEASING: 10,
  ARMSTATE_RESERVED: 255,
}

module.exports = ArmState;
