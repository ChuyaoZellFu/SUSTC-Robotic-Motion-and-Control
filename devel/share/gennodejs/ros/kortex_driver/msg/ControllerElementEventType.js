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

class ControllerElementEventType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerElementEventType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerElementEventType
    let len;
    let data = new ControllerElementEventType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerElementEventType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '30410419f52cbfa760b386b2213e4309';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CONTROLLER_ELEMENT_EVENT = 0
    
    uint32 AXIS_MOVED = 1
    
    uint32 BUTTON_DOWN = 2
    
    uint32 BUTTON_UP = 3
    
    uint32 BUTTON_CLICK = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerElementEventType(null);
    return resolved;
    }
};

// Constants for message
ControllerElementEventType.Constants = {
  UNSPECIFIED_CONTROLLER_ELEMENT_EVENT: 0,
  AXIS_MOVED: 1,
  BUTTON_DOWN: 2,
  BUTTON_UP: 3,
  BUTTON_CLICK: 4,
}

module.exports = ControllerElementEventType;
