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

class ControllerBehavior {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerBehavior
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerBehavior
    let len;
    let data = new ControllerBehavior(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerBehavior';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '12553caa8500dc92a4c4cfda3f89fc6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CONTROLLER_BEHAVIOR = 0
    
    uint32 CONTROLLER_BUTTON_DOWN = 1
    
    uint32 CONTROLLER_BUTTON_UP = 2
    
    uint32 CONTROLLER_AXIS_POSITIVE = 3
    
    uint32 CONTROLLER_AXIS_NEGATIVE = 4
    
    uint32 CONTROLLER_BUTTON_CLICK = 5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerBehavior(null);
    return resolved;
    }
};

// Constants for message
ControllerBehavior.Constants = {
  UNSPECIFIED_CONTROLLER_BEHAVIOR: 0,
  CONTROLLER_BUTTON_DOWN: 1,
  CONTROLLER_BUTTON_UP: 2,
  CONTROLLER_AXIS_POSITIVE: 3,
  CONTROLLER_AXIS_NEGATIVE: 4,
  CONTROLLER_BUTTON_CLICK: 5,
}

module.exports = ControllerBehavior;
