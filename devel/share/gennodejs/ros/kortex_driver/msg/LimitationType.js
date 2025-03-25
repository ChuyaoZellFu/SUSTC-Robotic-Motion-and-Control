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

class LimitationType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LimitationType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LimitationType
    let len;
    let data = new LimitationType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/LimitationType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86bbe126d2c218724be7709cc366b9d5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_LIMITATION = 0
    
    uint32 FORCE_LIMITATION = 1
    
    uint32 ACCELERATION_LIMITATION = 2
    
    uint32 VELOCITY_LIMITATION = 3
    
    uint32 TORQUE_LIMITATION = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LimitationType(null);
    return resolved;
    }
};

// Constants for message
LimitationType.Constants = {
  UNSPECIFIED_LIMITATION: 0,
  FORCE_LIMITATION: 1,
  ACCELERATION_LIMITATION: 2,
  VELOCITY_LIMITATION: 3,
  TORQUE_LIMITATION: 4,
}

module.exports = LimitationType;
