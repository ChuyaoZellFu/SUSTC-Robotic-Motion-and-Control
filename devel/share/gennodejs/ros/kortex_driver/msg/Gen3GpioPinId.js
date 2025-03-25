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

class Gen3GpioPinId {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gen3GpioPinId
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gen3GpioPinId
    let len;
    let data = new Gen3GpioPinId(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Gen3GpioPinId';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4129f3451b6d28fcb9b382446ece1a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_PIN = 0
    
    uint32 GPIO_PIN_B = 1
    
    uint32 GPIO_PIN_C = 2
    
    uint32 GPIO_PIN_D = 3
    
    uint32 GPIO_PIN_E = 4
    
    uint32 GPIO_PIN_G = 5
    
    uint32 GPIO_PIN_H = 6
    
    uint32 GPIO_PIN_I = 7
    
    uint32 GPIO_PIN_K = 8
    
    uint32 GPIO_PIN_N = 9
    
    uint32 GPIO_PIN_O = 10
    
    uint32 GPIO_PIN_S = 11
    
    uint32 GPIO_PIN_T = 12
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Gen3GpioPinId(null);
    return resolved;
    }
};

// Constants for message
Gen3GpioPinId.Constants = {
  UNSPECIFIED_PIN: 0,
  GPIO_PIN_B: 1,
  GPIO_PIN_C: 2,
  GPIO_PIN_D: 3,
  GPIO_PIN_E: 4,
  GPIO_PIN_G: 5,
  GPIO_PIN_H: 6,
  GPIO_PIN_I: 7,
  GPIO_PIN_K: 8,
  GPIO_PIN_N: 9,
  GPIO_PIN_O: 10,
  GPIO_PIN_S: 11,
  GPIO_PIN_T: 12,
}

module.exports = Gen3GpioPinId;
