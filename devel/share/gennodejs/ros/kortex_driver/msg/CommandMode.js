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

class CommandMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandMode
    let len;
    let data = new CommandMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CommandMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d888c2b80a47e457086f44a5dd3543e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 CYCLIC = 0
    
    uint32 ASYNC_CYCLIC_FLAGS = 1
    
    uint32 ASYNC = 2
    
    uint32 CYCLIC_JITTERCOMPENSATED_POSITION = 3
    
    uint32 CYCLIC_JITTERCOMPENSATED_VELOCITY = 4
    
    uint32 CYCLIC_JITTERCOMPENSATED_ACCELERATION = 5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandMode(null);
    return resolved;
    }
};

// Constants for message
CommandMode.Constants = {
  CYCLIC: 0,
  ASYNC_CYCLIC_FLAGS: 1,
  ASYNC: 2,
  CYCLIC_JITTERCOMPENSATED_POSITION: 3,
  CYCLIC_JITTERCOMPENSATED_VELOCITY: 4,
  CYCLIC_JITTERCOMPENSATED_ACCELERATION: 5,
}

module.exports = CommandMode;
