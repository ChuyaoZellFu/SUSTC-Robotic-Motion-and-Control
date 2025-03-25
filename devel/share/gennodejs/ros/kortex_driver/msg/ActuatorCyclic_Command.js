// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');

//-----------------------------------------------------------

class ActuatorCyclic_Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command_id = null;
      this.flags = null;
      this.position = null;
      this.velocity = null;
      this.torque_joint = null;
      this.current_motor = null;
    }
    else {
      if (initObj.hasOwnProperty('command_id')) {
        this.command_id = initObj.command_id
      }
      else {
        this.command_id = new ActuatorCyclic_MessageId();
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('torque_joint')) {
        this.torque_joint = initObj.torque_joint
      }
      else {
        this.torque_joint = 0.0;
      }
      if (initObj.hasOwnProperty('current_motor')) {
        this.current_motor = initObj.current_motor
      }
      else {
        this.current_motor = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActuatorCyclic_Command
    // Serialize message field [command_id]
    bufferOffset = ActuatorCyclic_MessageId.serialize(obj.command_id, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float32(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [torque_joint]
    bufferOffset = _serializer.float32(obj.torque_joint, buffer, bufferOffset);
    // Serialize message field [current_motor]
    bufferOffset = _serializer.float32(obj.current_motor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActuatorCyclic_Command
    let len;
    let data = new ActuatorCyclic_Command(null);
    // Deserialize message field [command_id]
    data.command_id = ActuatorCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [torque_joint]
    data.torque_joint = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_motor]
    data.current_motor = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActuatorCyclic_Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c79b9ac4bc4567cb00f9d7a7a89a1be9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ActuatorCyclic_MessageId command_id
    uint32 flags
    float32 position
    float32 velocity
    float32 torque_joint
    float32 current_motor
    ================================================================================
    MSG: kortex_driver/ActuatorCyclic_MessageId
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActuatorCyclic_Command(null);
    if (msg.command_id !== undefined) {
      resolved.command_id = ActuatorCyclic_MessageId.Resolve(msg.command_id)
    }
    else {
      resolved.command_id = new ActuatorCyclic_MessageId()
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.torque_joint !== undefined) {
      resolved.torque_joint = msg.torque_joint;
    }
    else {
      resolved.torque_joint = 0.0
    }

    if (msg.current_motor !== undefined) {
      resolved.current_motor = msg.current_motor;
    }
    else {
      resolved.current_motor = 0.0
    }

    return resolved;
    }
};

module.exports = ActuatorCyclic_Command;
