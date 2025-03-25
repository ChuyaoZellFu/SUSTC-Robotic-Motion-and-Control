// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');

//-----------------------------------------------------------

class InterconnectCyclic_Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command_id = null;
      this.flags = null;
      this.oneof_tool_command = null;
    }
    else {
      if (initObj.hasOwnProperty('command_id')) {
        this.command_id = initObj.command_id
      }
      else {
        this.command_id = new InterconnectCyclic_MessageId();
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('oneof_tool_command')) {
        this.oneof_tool_command = initObj.oneof_tool_command
      }
      else {
        this.oneof_tool_command = new InterconnectCyclic_Command_tool_command();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectCyclic_Command
    // Serialize message field [command_id]
    bufferOffset = InterconnectCyclic_MessageId.serialize(obj.command_id, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [oneof_tool_command]
    bufferOffset = InterconnectCyclic_Command_tool_command.serialize(obj.oneof_tool_command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectCyclic_Command
    let len;
    let data = new InterconnectCyclic_Command(null);
    // Deserialize message field [command_id]
    data.command_id = InterconnectCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [oneof_tool_command]
    data.oneof_tool_command = InterconnectCyclic_Command_tool_command.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += InterconnectCyclic_Command_tool_command.getMessageSize(object.oneof_tool_command);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectCyclic_Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4def369ad2d800b446113efeb710b701';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    InterconnectCyclic_MessageId command_id
    uint32 flags
    InterconnectCyclic_Command_tool_command oneof_tool_command
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_Command_tool_command
    
    GripperCyclic_Command[] gripper_command
    ================================================================================
    MSG: kortex_driver/GripperCyclic_Command
    
    GripperCyclic_MessageId command_id
    uint32 flags
    MotorCommand[] motor_cmd
    ================================================================================
    MSG: kortex_driver/GripperCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/MotorCommand
    
    uint32 motor_id
    float32 position
    float32 velocity
    float32 force
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InterconnectCyclic_Command(null);
    if (msg.command_id !== undefined) {
      resolved.command_id = InterconnectCyclic_MessageId.Resolve(msg.command_id)
    }
    else {
      resolved.command_id = new InterconnectCyclic_MessageId()
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.oneof_tool_command !== undefined) {
      resolved.oneof_tool_command = InterconnectCyclic_Command_tool_command.Resolve(msg.oneof_tool_command)
    }
    else {
      resolved.oneof_tool_command = new InterconnectCyclic_Command_tool_command()
    }

    return resolved;
    }
};

module.exports = InterconnectCyclic_Command;
