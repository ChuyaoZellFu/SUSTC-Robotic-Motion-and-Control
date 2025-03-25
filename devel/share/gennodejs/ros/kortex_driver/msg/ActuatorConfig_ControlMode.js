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

class ActuatorConfig_ControlMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActuatorConfig_ControlMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActuatorConfig_ControlMode
    let len;
    let data = new ActuatorConfig_ControlMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActuatorConfig_ControlMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb12ce17d437184d7baad58c2fe8b556';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 NONE = 0
    
    uint32 POSITION = 1
    
    uint32 VELOCITY = 2
    
    uint32 TORQUE = 3
    
    uint32 CURRENT = 4
    
    uint32 CUSTOM = 5
    
    uint32 TORQUE_HIGH_VELOCITY = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActuatorConfig_ControlMode(null);
    return resolved;
    }
};

// Constants for message
ActuatorConfig_ControlMode.Constants = {
  NONE: 0,
  POSITION: 1,
  VELOCITY: 2,
  TORQUE: 3,
  CURRENT: 4,
  CUSTOM: 5,
  TORQUE_HIGH_VELOCITY: 6,
}

module.exports = ActuatorConfig_ControlMode;
