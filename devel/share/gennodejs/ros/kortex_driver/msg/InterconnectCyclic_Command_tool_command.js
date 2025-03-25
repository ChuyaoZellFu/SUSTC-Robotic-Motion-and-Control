// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GripperCyclic_Command = require('./GripperCyclic_Command.js');

//-----------------------------------------------------------

class InterconnectCyclic_Command_tool_command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gripper_command = null;
    }
    else {
      if (initObj.hasOwnProperty('gripper_command')) {
        this.gripper_command = initObj.gripper_command
      }
      else {
        this.gripper_command = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectCyclic_Command_tool_command
    // Serialize message field [gripper_command]
    // Serialize the length for message field [gripper_command]
    bufferOffset = _serializer.uint32(obj.gripper_command.length, buffer, bufferOffset);
    obj.gripper_command.forEach((val) => {
      bufferOffset = GripperCyclic_Command.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectCyclic_Command_tool_command
    let len;
    let data = new InterconnectCyclic_Command_tool_command(null);
    // Deserialize message field [gripper_command]
    // Deserialize array length for message field [gripper_command]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gripper_command = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gripper_command[i] = GripperCyclic_Command.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gripper_command.forEach((val) => {
      length += GripperCyclic_Command.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectCyclic_Command_tool_command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb4b9ede5a008a07d439d113030c34d0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new InterconnectCyclic_Command_tool_command(null);
    if (msg.gripper_command !== undefined) {
      resolved.gripper_command = new Array(msg.gripper_command.length);
      for (let i = 0; i < resolved.gripper_command.length; ++i) {
        resolved.gripper_command[i] = GripperCyclic_Command.Resolve(msg.gripper_command[i]);
      }
    }
    else {
      resolved.gripper_command = []
    }

    return resolved;
    }
};

module.exports = InterconnectCyclic_Command_tool_command;
