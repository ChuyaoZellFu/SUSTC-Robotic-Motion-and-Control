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

class LedState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LedState
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LedState
    let len;
    let data = new LedState(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/LedState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd10da8147549713ab2f5b5d7555fb6b6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_LED_STATE = 0
    
    uint32 LED_OFF = 1
    
    uint32 LED_PULSE = 2
    
    uint32 LED_ON = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LedState(null);
    return resolved;
    }
};

// Constants for message
LedState.Constants = {
  UNSPECIFIED_LED_STATE: 0,
  LED_OFF: 1,
  LED_PULSE: 2,
  LED_ON: 3,
}

module.exports = LedState;
