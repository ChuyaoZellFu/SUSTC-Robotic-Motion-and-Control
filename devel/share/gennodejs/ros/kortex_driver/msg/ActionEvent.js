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

class ActionEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionEvent
    let len;
    let data = new ActionEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActionEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34076dc876693564b13adf0d712f14bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_ACTION_EVENT = 0
    
    uint32 ACTION_END = 1
    
    uint32 ACTION_ABORT = 2
    
    uint32 ACTION_PAUSE = 3
    
    uint32 ACTION_START = 4
    
    uint32 ACTION_PREPROCESS_START = 5
    
    uint32 ACTION_PREPROCESS_ABORT = 6
    
    uint32 ACTION_PREPROCESS_END = 7
    
    uint32 ACTION_POSTPROCESS_START = 8
    
    uint32 ACTION_POSTPROCESS_ABORT = 9
    
    uint32 ACTION_POSTPROCESS_END = 10
    
    uint32 ACTION_FEEDBACK = 11
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionEvent(null);
    return resolved;
    }
};

// Constants for message
ActionEvent.Constants = {
  UNSPECIFIED_ACTION_EVENT: 0,
  ACTION_END: 1,
  ACTION_ABORT: 2,
  ACTION_PAUSE: 3,
  ACTION_START: 4,
  ACTION_PREPROCESS_START: 5,
  ACTION_PREPROCESS_ABORT: 6,
  ACTION_PREPROCESS_END: 7,
  ACTION_POSTPROCESS_START: 8,
  ACTION_POSTPROCESS_ABORT: 9,
  ACTION_POSTPROCESS_END: 10,
  ACTION_FEEDBACK: 11,
}

module.exports = ActionEvent;
