// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let MotorCommand = require('./MotorCommand.js');

//-----------------------------------------------------------

class GripperCyclic_Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command_id = null;
      this.flags = null;
      this.motor_cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('command_id')) {
        this.command_id = initObj.command_id
      }
      else {
        this.command_id = new GripperCyclic_MessageId();
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('motor_cmd')) {
        this.motor_cmd = initObj.motor_cmd
      }
      else {
        this.motor_cmd = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperCyclic_Command
    // Serialize message field [command_id]
    bufferOffset = GripperCyclic_MessageId.serialize(obj.command_id, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [motor_cmd]
    // Serialize the length for message field [motor_cmd]
    bufferOffset = _serializer.uint32(obj.motor_cmd.length, buffer, bufferOffset);
    obj.motor_cmd.forEach((val) => {
      bufferOffset = MotorCommand.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperCyclic_Command
    let len;
    let data = new GripperCyclic_Command(null);
    // Deserialize message field [command_id]
    data.command_id = GripperCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [motor_cmd]
    // Deserialize array length for message field [motor_cmd]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.motor_cmd = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motor_cmd[i] = MotorCommand.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.motor_cmd.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GripperCyclic_Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa88200f31c06368a2a9758c0985b419';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new GripperCyclic_Command(null);
    if (msg.command_id !== undefined) {
      resolved.command_id = GripperCyclic_MessageId.Resolve(msg.command_id)
    }
    else {
      resolved.command_id = new GripperCyclic_MessageId()
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.motor_cmd !== undefined) {
      resolved.motor_cmd = new Array(msg.motor_cmd.length);
      for (let i = 0; i < resolved.motor_cmd.length; ++i) {
        resolved.motor_cmd[i] = MotorCommand.Resolve(msg.motor_cmd[i]);
      }
    }
    else {
      resolved.motor_cmd = []
    }

    return resolved;
    }
};

module.exports = GripperCyclic_Command;
