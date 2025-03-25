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

class ActuatorConfig_SafetyLimitType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActuatorConfig_SafetyLimitType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActuatorConfig_SafetyLimitType
    let len;
    let data = new ActuatorConfig_SafetyLimitType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActuatorConfig_SafetyLimitType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2886a0a967849f2573bf0fd798659a83';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 MAXIMAL_LIMIT = 0
    
    uint32 MINIMAL_LIMIT = 1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActuatorConfig_SafetyLimitType(null);
    return resolved;
    }
};

// Constants for message
ActuatorConfig_SafetyLimitType.Constants = {
  MAXIMAL_LIMIT: 0,
  MINIMAL_LIMIT: 1,
}

module.exports = ActuatorConfig_SafetyLimitType;
