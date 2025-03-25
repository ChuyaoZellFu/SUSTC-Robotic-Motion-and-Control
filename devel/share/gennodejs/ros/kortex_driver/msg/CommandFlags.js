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

class CommandFlags {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandFlags
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandFlags
    let len;
    let data = new CommandFlags(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CommandFlags';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e51ff7d8ad00c576e3aaa1ef0c879bf9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 NO_COMMAND = 0
    
    uint32 SERVO_ENABLE = 1
    
    uint32 BRAKE_DISABLE = 2
    
    uint32 CLEAR_MAJOR_FAULT = 4
    
    uint32 CLEAR_MINOR_FAULT = 8
    
    uint32 PROTECTIVE_STOP = 16
    
    uint32 FORCE_BRAKE_RELEASE = 32
    
    uint32 IGNORE = 64
    
    uint32 LOW_GAINS = 128
    
    uint32 LED_0 = 256
    
    uint32 LED_1 = 512
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandFlags(null);
    return resolved;
    }
};

// Constants for message
CommandFlags.Constants = {
  NO_COMMAND: 0,
  SERVO_ENABLE: 1,
  BRAKE_DISABLE: 2,
  CLEAR_MAJOR_FAULT: 4,
  CLEAR_MINOR_FAULT: 8,
  PROTECTIVE_STOP: 16,
  FORCE_BRAKE_RELEASE: 32,
  IGNORE: 64,
  LOW_GAINS: 128,
  LED_0: 256,
  LED_1: 512,
}

module.exports = CommandFlags;
