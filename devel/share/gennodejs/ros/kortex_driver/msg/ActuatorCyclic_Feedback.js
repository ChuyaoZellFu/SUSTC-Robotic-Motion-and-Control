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

class ActuatorCyclic_Feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.feedback_id = null;
      this.status_flags = null;
      this.jitter_comm = null;
      this.position = null;
      this.velocity = null;
      this.torque = null;
      this.current_motor = null;
      this.voltage = null;
      this.temperature_motor = null;
      this.temperature_core = null;
      this.fault_bank_a = null;
      this.fault_bank_b = null;
      this.warning_bank_a = null;
      this.warning_bank_b = null;
    }
    else {
      if (initObj.hasOwnProperty('feedback_id')) {
        this.feedback_id = initObj.feedback_id
      }
      else {
        this.feedback_id = new ActuatorCyclic_MessageId();
      }
      if (initObj.hasOwnProperty('status_flags')) {
        this.status_flags = initObj.status_flags
      }
      else {
        this.status_flags = 0;
      }
      if (initObj.hasOwnProperty('jitter_comm')) {
        this.jitter_comm = initObj.jitter_comm
      }
      else {
        this.jitter_comm = 0;
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
      if (initObj.hasOwnProperty('torque')) {
        this.torque = initObj.torque
      }
      else {
        this.torque = 0.0;
      }
      if (initObj.hasOwnProperty('current_motor')) {
        this.current_motor = initObj.current_motor
      }
      else {
        this.current_motor = 0.0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_motor')) {
        this.temperature_motor = initObj.temperature_motor
      }
      else {
        this.temperature_motor = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_core')) {
        this.temperature_core = initObj.temperature_core
      }
      else {
        this.temperature_core = 0.0;
      }
      if (initObj.hasOwnProperty('fault_bank_a')) {
        this.fault_bank_a = initObj.fault_bank_a
      }
      else {
        this.fault_bank_a = 0;
      }
      if (initObj.hasOwnProperty('fault_bank_b')) {
        this.fault_bank_b = initObj.fault_bank_b
      }
      else {
        this.fault_bank_b = 0;
      }
      if (initObj.hasOwnProperty('warning_bank_a')) {
        this.warning_bank_a = initObj.warning_bank_a
      }
      else {
        this.warning_bank_a = 0;
      }
      if (initObj.hasOwnProperty('warning_bank_b')) {
        this.warning_bank_b = initObj.warning_bank_b
      }
      else {
        this.warning_bank_b = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActuatorCyclic_Feedback
    // Serialize message field [feedback_id]
    bufferOffset = ActuatorCyclic_MessageId.serialize(obj.feedback_id, buffer, bufferOffset);
    // Serialize message field [status_flags]
    bufferOffset = _serializer.uint32(obj.status_flags, buffer, bufferOffset);
    // Serialize message field [jitter_comm]
    bufferOffset = _serializer.uint32(obj.jitter_comm, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float32(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [torque]
    bufferOffset = _serializer.float32(obj.torque, buffer, bufferOffset);
    // Serialize message field [current_motor]
    bufferOffset = _serializer.float32(obj.current_motor, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float32(obj.voltage, buffer, bufferOffset);
    // Serialize message field [temperature_motor]
    bufferOffset = _serializer.float32(obj.temperature_motor, buffer, bufferOffset);
    // Serialize message field [temperature_core]
    bufferOffset = _serializer.float32(obj.temperature_core, buffer, bufferOffset);
    // Serialize message field [fault_bank_a]
    bufferOffset = _serializer.uint32(obj.fault_bank_a, buffer, bufferOffset);
    // Serialize message field [fault_bank_b]
    bufferOffset = _serializer.uint32(obj.fault_bank_b, buffer, bufferOffset);
    // Serialize message field [warning_bank_a]
    bufferOffset = _serializer.uint32(obj.warning_bank_a, buffer, bufferOffset);
    // Serialize message field [warning_bank_b]
    bufferOffset = _serializer.uint32(obj.warning_bank_b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActuatorCyclic_Feedback
    let len;
    let data = new ActuatorCyclic_Feedback(null);
    // Deserialize message field [feedback_id]
    data.feedback_id = ActuatorCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [status_flags]
    data.status_flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [jitter_comm]
    data.jitter_comm = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [torque]
    data.torque = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_motor]
    data.current_motor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature_motor]
    data.temperature_motor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature_core]
    data.temperature_core = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fault_bank_a]
    data.fault_bank_a = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [fault_bank_b]
    data.fault_bank_b = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning_bank_a]
    data.warning_bank_a = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning_bank_b]
    data.warning_bank_b = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActuatorCyclic_Feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '299d9bdfeb28700b38cf7f19f730d6c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ActuatorCyclic_MessageId feedback_id
    uint32 status_flags
    uint32 jitter_comm
    float32 position
    float32 velocity
    float32 torque
    float32 current_motor
    float32 voltage
    float32 temperature_motor
    float32 temperature_core
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
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
    const resolved = new ActuatorCyclic_Feedback(null);
    if (msg.feedback_id !== undefined) {
      resolved.feedback_id = ActuatorCyclic_MessageId.Resolve(msg.feedback_id)
    }
    else {
      resolved.feedback_id = new ActuatorCyclic_MessageId()
    }

    if (msg.status_flags !== undefined) {
      resolved.status_flags = msg.status_flags;
    }
    else {
      resolved.status_flags = 0
    }

    if (msg.jitter_comm !== undefined) {
      resolved.jitter_comm = msg.jitter_comm;
    }
    else {
      resolved.jitter_comm = 0
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

    if (msg.torque !== undefined) {
      resolved.torque = msg.torque;
    }
    else {
      resolved.torque = 0.0
    }

    if (msg.current_motor !== undefined) {
      resolved.current_motor = msg.current_motor;
    }
    else {
      resolved.current_motor = 0.0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.temperature_motor !== undefined) {
      resolved.temperature_motor = msg.temperature_motor;
    }
    else {
      resolved.temperature_motor = 0.0
    }

    if (msg.temperature_core !== undefined) {
      resolved.temperature_core = msg.temperature_core;
    }
    else {
      resolved.temperature_core = 0.0
    }

    if (msg.fault_bank_a !== undefined) {
      resolved.fault_bank_a = msg.fault_bank_a;
    }
    else {
      resolved.fault_bank_a = 0
    }

    if (msg.fault_bank_b !== undefined) {
      resolved.fault_bank_b = msg.fault_bank_b;
    }
    else {
      resolved.fault_bank_b = 0
    }

    if (msg.warning_bank_a !== undefined) {
      resolved.warning_bank_a = msg.warning_bank_a;
    }
    else {
      resolved.warning_bank_a = 0
    }

    if (msg.warning_bank_b !== undefined) {
      resolved.warning_bank_b = msg.warning_bank_b;
    }
    else {
      resolved.warning_bank_b = 0
    }

    return resolved;
    }
};

module.exports = ActuatorCyclic_Feedback;
