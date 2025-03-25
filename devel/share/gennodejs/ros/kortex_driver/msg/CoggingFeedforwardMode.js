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

class CoggingFeedforwardMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CoggingFeedforwardMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CoggingFeedforwardMode
    let len;
    let data = new CoggingFeedforwardMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CoggingFeedforwardMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '040e60ae33515a7c8082e377632750e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 FEEDFORWARD_OFF = 0
    
    uint32 FEEDFORWARD_ADAPTIVE = 1
    
    uint32 FEEDFORWARD_CALIBRATED = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CoggingFeedforwardMode(null);
    return resolved;
    }
};

// Constants for message
CoggingFeedforwardMode.Constants = {
  FEEDFORWARD_OFF: 0,
  FEEDFORWARD_ADAPTIVE: 1,
  FEEDFORWARD_CALIBRATED: 2,
}

module.exports = CoggingFeedforwardMode;
