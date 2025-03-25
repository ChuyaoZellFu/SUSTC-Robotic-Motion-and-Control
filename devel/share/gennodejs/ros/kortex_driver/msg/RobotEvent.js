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

class RobotEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotEvent
    let len;
    let data = new RobotEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RobotEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7edc4e787111520cc97803b60a159bc5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_ROBOT_EVENT = 0
    
    uint32 ARM_CONNECTED = 1
    
    uint32 ARM_DISCONNECTED = 2
    
    uint32 TOOL_CONNECTED = 5
    
    uint32 TOOL_DISCONNECTED = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotEvent(null);
    return resolved;
    }
};

// Constants for message
RobotEvent.Constants = {
  UNSPECIFIED_ROBOT_EVENT: 0,
  ARM_CONNECTED: 1,
  ARM_DISCONNECTED: 2,
  TOOL_CONNECTED: 5,
  TOOL_DISCONNECTED: 6,
}

module.exports = RobotEvent;
