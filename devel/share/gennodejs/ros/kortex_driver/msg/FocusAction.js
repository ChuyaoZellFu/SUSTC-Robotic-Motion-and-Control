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

class FocusAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FocusAction
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FocusAction
    let len;
    let data = new FocusAction(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FocusAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16232e452a5de2238b9046f807acd748';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 FOCUSACTION_UNSPECIFIED = 0
    
    uint32 FOCUSACTION_START_CONTINUOUS_FOCUS = 1
    
    uint32 FOCUSACTION_PAUSE_CONTINUOUS_FOCUS = 2
    
    uint32 FOCUSACTION_FOCUS_NOW = 3
    
    uint32 FOCUSACTION_DISABLE_FOCUS = 4
    
    uint32 FOCUSACTION_SET_FOCUS_POINT = 5
    
    uint32 FOCUSACTION_SET_MANUAL_FOCUS = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FocusAction(null);
    return resolved;
    }
};

// Constants for message
FocusAction.Constants = {
  FOCUSACTION_UNSPECIFIED: 0,
  FOCUSACTION_START_CONTINUOUS_FOCUS: 1,
  FOCUSACTION_PAUSE_CONTINUOUS_FOCUS: 2,
  FOCUSACTION_FOCUS_NOW: 3,
  FOCUSACTION_DISABLE_FOCUS: 4,
  FOCUSACTION_SET_FOCUS_POINT: 5,
  FOCUSACTION_SET_MANUAL_FOCUS: 6,
}

module.exports = FocusAction;
