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

class CommandModeInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('command_mode')) {
        this.command_mode = initObj.command_mode
      }
      else {
        this.command_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandModeInformation
    // Serialize message field [command_mode]
    bufferOffset = _serializer.uint32(obj.command_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandModeInformation
    let len;
    let data = new CommandModeInformation(null);
    // Deserialize message field [command_mode]
    data.command_mode = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CommandModeInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d0d1fb206a87d77cdd9700eb23ea5cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 command_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandModeInformation(null);
    if (msg.command_mode !== undefined) {
      resolved.command_mode = msg.command_mode;
    }
    else {
      resolved.command_mode = 0
    }

    return resolved;
    }
};

module.exports = CommandModeInformation;
