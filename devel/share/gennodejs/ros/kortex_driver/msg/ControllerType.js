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

class ControllerType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerType
    let len;
    let data = new ControllerType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '482367554f194246d15dc5fb1ede90ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CONTROLLER_TYPE = 0
    
    uint32 XBOX_CONTROLLER = 1
    
    uint32 WRIST_CONTROLLER = 2
    
    uint32 BASIC_JOYSTICK_CONTROLLER = 3
    
    uint32 BASE_GPIO_CONTROLLER = 4
    
    uint32 GPIO_JOYSTICK = 5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerType(null);
    return resolved;
    }
};

// Constants for message
ControllerType.Constants = {
  UNSPECIFIED_CONTROLLER_TYPE: 0,
  XBOX_CONTROLLER: 1,
  WRIST_CONTROLLER: 2,
  BASIC_JOYSTICK_CONTROLLER: 3,
  BASE_GPIO_CONTROLLER: 4,
  GPIO_JOYSTICK: 5,
}

module.exports = ControllerType;
