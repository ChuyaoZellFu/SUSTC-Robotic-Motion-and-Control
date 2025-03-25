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

class GpioAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioAction
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioAction
    let len;
    let data = new GpioAction(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7271e7564b2e393d951b0684902edec6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_GPIO_ACTION = 0
    
    uint32 GPIOACTION_SET = 1
    
    uint32 GPIOACTION_CLEAR = 2
    
    uint32 GPIOACTION_PULSE_HIGH = 3
    
    uint32 GPIOACTION_PULSE_LOW = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioAction(null);
    return resolved;
    }
};

// Constants for message
GpioAction.Constants = {
  UNSPECIFIED_GPIO_ACTION: 0,
  GPIOACTION_SET: 1,
  GPIOACTION_CLEAR: 2,
  GPIOACTION_PULSE_HIGH: 3,
  GPIOACTION_PULSE_LOW: 4,
}

module.exports = GpioAction;
