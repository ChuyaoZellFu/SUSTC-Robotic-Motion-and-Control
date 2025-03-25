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

class Xbox360AnalogInputIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Xbox360AnalogInputIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Xbox360AnalogInputIdentifier
    let len;
    let data = new Xbox360AnalogInputIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Xbox360AnalogInputIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bdf16a9285171f4643549a6ec207283d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_XBOX360_ANALOG = 0
    
    uint32 XBOX360_THUMB_LEFT_X = 1
    
    uint32 XBOX360_THUMB_LEFT_Y = 2
    
    uint32 XBOX360_THUMB_RIGHT_X = 3
    
    uint32 XBOX360_THUMB_RIGHT_Y = 4
    
    uint32 XBOX360_TRIGGER_LEFT = 5
    
    uint32 XBOX360_TRIGGER_RIGHT = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Xbox360AnalogInputIdentifier(null);
    return resolved;
    }
};

// Constants for message
Xbox360AnalogInputIdentifier.Constants = {
  UNSPECIFIED_XBOX360_ANALOG: 0,
  XBOX360_THUMB_LEFT_X: 1,
  XBOX360_THUMB_LEFT_Y: 2,
  XBOX360_THUMB_RIGHT_X: 3,
  XBOX360_THUMB_RIGHT_Y: 4,
  XBOX360_TRIGGER_LEFT: 5,
  XBOX360_TRIGGER_RIGHT: 6,
}

module.exports = Xbox360AnalogInputIdentifier;
