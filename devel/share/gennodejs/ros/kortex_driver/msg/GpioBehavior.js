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

class GpioBehavior {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioBehavior
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioBehavior
    let len;
    let data = new GpioBehavior(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioBehavior';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a285ed144abb1d2ba02c8fa81ee67296';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_GPIO_BEHAVIOR = 0
    
    uint32 GPIO_FALLING = 1
    
    uint32 GPIO_RISING = 2
    
    uint32 GPIO_PULSE_LOW = 3
    
    uint32 GPIO_PULSE_HIGH = 4
    
    uint32 GPIO_ANALOG_CHANGE = 5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioBehavior(null);
    return resolved;
    }
};

// Constants for message
GpioBehavior.Constants = {
  UNSPECIFIED_GPIO_BEHAVIOR: 0,
  GPIO_FALLING: 1,
  GPIO_RISING: 2,
  GPIO_PULSE_LOW: 3,
  GPIO_PULSE_HIGH: 4,
  GPIO_ANALOG_CHANGE: 5,
}

module.exports = GpioBehavior;
