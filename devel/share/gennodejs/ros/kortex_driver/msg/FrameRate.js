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

class FrameRate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrameRate
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrameRate
    let len;
    let data = new FrameRate(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FrameRate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9400397c1a84f4718dc33b3b7f3b5df4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 FRAMERATE_UNSPECIFIED = 0
    
    uint32 FRAMERATE_6_FPS = 1
    
    uint32 FRAMERATE_15_FPS = 2
    
    uint32 FRAMERATE_30_FPS = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FrameRate(null);
    return resolved;
    }
};

// Constants for message
FrameRate.Constants = {
  FRAMERATE_UNSPECIFIED: 0,
  FRAMERATE_6_FPS: 1,
  FRAMERATE_15_FPS: 2,
  FRAMERATE_30_FPS: 3,
}

module.exports = FrameRate;
