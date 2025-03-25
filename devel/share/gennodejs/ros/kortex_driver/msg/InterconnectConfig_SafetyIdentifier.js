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

class InterconnectConfig_SafetyIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectConfig_SafetyIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectConfig_SafetyIdentifier
    let len;
    let data = new InterconnectConfig_SafetyIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectConfig_SafetyIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b36683ad678ea1db45ab5eb82312ac0b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_INTERCONNECT_SAFETY_IDENTIFIER = 0
    
    uint32 MAXIMUM_MOTOR_CURRENT = 1
    
    uint32 MAXIMUM_VOLTAGE = 2
    
    uint32 MINIMUM_VOLTAGE = 4
    
    uint32 MAXIMUM_MOTOR_TEMPERATURE = 8
    
    uint32 MAXIMUM_CORE_TEMPERATURE = 16
    
    uint32 NON_VOLATILE_MEMORY_CORRUPTED = 32
    
    uint32 EMERGENCY_LINE_ASSERTED = 64
    
    uint32 COMMUNICATION_TICK_LOST = 128
    
    uint32 WATCHDOG_TRIGGERED = 256
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InterconnectConfig_SafetyIdentifier(null);
    return resolved;
    }
};

// Constants for message
InterconnectConfig_SafetyIdentifier.Constants = {
  UNSPECIFIED_INTERCONNECT_SAFETY_IDENTIFIER: 0,
  MAXIMUM_MOTOR_CURRENT: 1,
  MAXIMUM_VOLTAGE: 2,
  MINIMUM_VOLTAGE: 4,
  MAXIMUM_MOTOR_TEMPERATURE: 8,
  MAXIMUM_CORE_TEMPERATURE: 16,
  NON_VOLATILE_MEMORY_CORRUPTED: 32,
  EMERGENCY_LINE_ASSERTED: 64,
  COMMUNICATION_TICK_LOST: 128,
  WATCHDOG_TRIGGERED: 256,
}

module.exports = InterconnectConfig_SafetyIdentifier;
