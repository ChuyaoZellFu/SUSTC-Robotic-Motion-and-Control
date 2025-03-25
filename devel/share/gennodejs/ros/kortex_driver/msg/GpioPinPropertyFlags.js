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

class GpioPinPropertyFlags {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioPinPropertyFlags
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioPinPropertyFlags
    let len;
    let data = new GpioPinPropertyFlags(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioPinPropertyFlags';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '531958ae411036543a3b84e9b7f802d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 GPIOPROPERTY_UNKNOWN = 0
    
    uint32 GPIOPROPERTY_INPUT = 1
    
    uint32 GPIOPROPERTY_OUTPUT = 2
    
    uint32 GPIOPROPERTY_ANALOG = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioPinPropertyFlags(null);
    return resolved;
    }
};

// Constants for message
GpioPinPropertyFlags.Constants = {
  GPIOPROPERTY_UNKNOWN: 0,
  GPIOPROPERTY_INPUT: 1,
  GPIOPROPERTY_OUTPUT: 2,
  GPIOPROPERTY_ANALOG: 4,
}

module.exports = GpioPinPropertyFlags;
