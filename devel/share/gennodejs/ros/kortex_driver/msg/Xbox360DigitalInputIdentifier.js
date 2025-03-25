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

class Xbox360DigitalInputIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Xbox360DigitalInputIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Xbox360DigitalInputIdentifier
    let len;
    let data = new Xbox360DigitalInputIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Xbox360DigitalInputIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b1197219b8c5e86e78fa24aefadcb86e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_XBOX360_DIGITAL = 0
    
    uint32 XBOX360_PAD_UP = 1
    
    uint32 XBOX360_PAD_DOWN = 2
    
    uint32 XBOX360_PAD_LEFT = 3
    
    uint32 XBOX360_PAD_RIGHT = 4
    
    uint32 XBOX360_FILE_BUTTON_START = 5
    
    uint32 XBOX360_DOCUMENT_BUTTON_BACK = 6
    
    uint32 XBOX360_LEFT_THUMB_BUTTON = 7
    
    uint32 XBOX360_RIGHT_THUMB_BUTTON = 8
    
    uint32 XBOX360_LEFT_SHOULDER = 9
    
    uint32 XBOX360_RIGHT_SHOULDER = 10
    
    uint32 XBOX360_BUTTON_A = 13
    
    uint32 XBOX360_BUTTON_B = 14
    
    uint32 XBOX360_BUTTON_X = 15
    
    uint32 XBOX360_BUTTON_Y = 16
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Xbox360DigitalInputIdentifier(null);
    return resolved;
    }
};

// Constants for message
Xbox360DigitalInputIdentifier.Constants = {
  UNSPECIFIED_XBOX360_DIGITAL: 0,
  XBOX360_PAD_UP: 1,
  XBOX360_PAD_DOWN: 2,
  XBOX360_PAD_LEFT: 3,
  XBOX360_PAD_RIGHT: 4,
  XBOX360_FILE_BUTTON_START: 5,
  XBOX360_DOCUMENT_BUTTON_BACK: 6,
  XBOX360_LEFT_THUMB_BUTTON: 7,
  XBOX360_RIGHT_THUMB_BUTTON: 8,
  XBOX360_LEFT_SHOULDER: 9,
  XBOX360_RIGHT_SHOULDER: 10,
  XBOX360_BUTTON_A: 13,
  XBOX360_BUTTON_B: 14,
  XBOX360_BUTTON_X: 15,
  XBOX360_BUTTON_Y: 16,
}

module.exports = Xbox360DigitalInputIdentifier;
