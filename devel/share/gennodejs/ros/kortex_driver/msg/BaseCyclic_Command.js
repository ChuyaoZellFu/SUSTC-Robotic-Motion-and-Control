// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ActuatorCommand = require('./ActuatorCommand.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');

//-----------------------------------------------------------

class BaseCyclic_Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.actuators = null;
      this.interconnect = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = 0;
      }
      if (initObj.hasOwnProperty('actuators')) {
        this.actuators = initObj.actuators
      }
      else {
        this.actuators = [];
      }
      if (initObj.hasOwnProperty('interconnect')) {
        this.interconnect = initObj.interconnect
      }
      else {
        this.interconnect = new InterconnectCyclic_Command();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseCyclic_Command
    // Serialize message field [frame_id]
    bufferOffset = _serializer.uint32(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [actuators]
    // Serialize the length for message field [actuators]
    bufferOffset = _serializer.uint32(obj.actuators.length, buffer, bufferOffset);
    obj.actuators.forEach((val) => {
      bufferOffset = ActuatorCommand.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [interconnect]
    bufferOffset = InterconnectCyclic_Command.serialize(obj.interconnect, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseCyclic_Command
    let len;
    let data = new BaseCyclic_Command(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [actuators]
    // Deserialize array length for message field [actuators]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.actuators = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.actuators[i] = ActuatorCommand.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [interconnect]
    data.interconnect = InterconnectCyclic_Command.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.actuators.length;
    length += InterconnectCyclic_Command.getMessageSize(object.interconnect);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BaseCyclic_Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '79e9d628025fc54837a4192c98fc5b9f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 frame_id
    ActuatorCommand[] actuators
    InterconnectCyclic_Command interconnect
    ================================================================================
    MSG: kortex_driver/ActuatorCommand
    
    uint32 command_id
    uint32 flags
    float32 position
    float32 velocity
    float32 torque_joint
    float32 current_motor
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_Command
    
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
    const resolved = new BaseCyclic_Command(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = 0
    }

    if (msg.actuators !== undefined) {
      resolved.actuators = new Array(msg.actuators.length);
      for (let i = 0; i < resolved.actuators.length; ++i) {
        resolved.actuators[i] = ActuatorCommand.Resolve(msg.actuators[i]);
      }
    }
    else {
      resolved.actuators = []
    }

    if (msg.interconnect !== undefined) {
      resolved.interconnect = InterconnectCyclic_Command.Resolve(msg.interconnect)
    }
    else {
      resolved.interconnect = new InterconnectCyclic_Command()
    }

    return resolved;
    }
};

module.exports = BaseCyclic_Command;
