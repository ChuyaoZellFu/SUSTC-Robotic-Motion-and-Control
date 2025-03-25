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

class Resolution {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Resolution
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Resolution
    let len;
    let data = new Resolution(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Resolution';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa3fc86ed5b2ab08e6ab3385eebb39de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 RESOLUTION_UNSPECIFIED = 0
    
    uint32 RESOLUTION_320x240 = 1
    
    uint32 RESOLUTION_424x240 = 2
    
    uint32 RESOLUTION_480x270 = 3
    
    uint32 RESOLUTION_640x480 = 4
    
    uint32 RESOLUTION_1280x720 = 5
    
    uint32 RESOLUTION_1920x1080 = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Resolution(null);
    return resolved;
    }
};

// Constants for message
Resolution.Constants = {
  RESOLUTION_UNSPECIFIED: 0,
  RESOLUTION_320X240: 1,
  RESOLUTION_424X240: 2,
  RESOLUTION_480X270: 3,
  RESOLUTION_640X480: 4,
  RESOLUTION_1280X720: 5,
  RESOLUTION_1920X1080: 6,
}

module.exports = Resolution;
