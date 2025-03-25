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
let MotorFeedback = require('./MotorFeedback.js');

//-----------------------------------------------------------

class GripperCyclic_Feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.feedback_id = null;
      this.status_flags = null;
      this.fault_bank_a = null;
      this.fault_bank_b = null;
      this.warning_bank_a = null;
      this.warning_bank_b = null;
      this.motor = null;
    }
    else {
      if (initObj.hasOwnProperty('feedback_id')) {
        this.feedback_id = initObj.feedback_id
      }
      else {
        this.feedback_id = new GripperCyclic_MessageId();
      }
      if (initObj.hasOwnProperty('status_flags')) {
        this.status_flags = initObj.status_flags
      }
      else {
        this.status_flags = 0;
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
      if (initObj.hasOwnProperty('motor')) {
        this.motor = initObj.motor
      }
      else {
        this.motor = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperCyclic_Feedback
    // Serialize message field [feedback_id]
    bufferOffset = GripperCyclic_MessageId.serialize(obj.feedback_id, buffer, bufferOffset);
    // Serialize message field [status_flags]
    bufferOffset = _serializer.uint32(obj.status_flags, buffer, bufferOffset);
    // Serialize message field [fault_bank_a]
    bufferOffset = _serializer.uint32(obj.fault_bank_a, buffer, bufferOffset);
    // Serialize message field [fault_bank_b]
    bufferOffset = _serializer.uint32(obj.fault_bank_b, buffer, bufferOffset);
    // Serialize message field [warning_bank_a]
    bufferOffset = _serializer.uint32(obj.warning_bank_a, buffer, bufferOffset);
    // Serialize message field [warning_bank_b]
    bufferOffset = _serializer.uint32(obj.warning_bank_b, buffer, bufferOffset);
    // Serialize message field [motor]
    // Serialize the length for message field [motor]
    bufferOffset = _serializer.uint32(obj.motor.length, buffer, bufferOffset);
    obj.motor.forEach((val) => {
      bufferOffset = MotorFeedback.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperCyclic_Feedback
    let len;
    let data = new GripperCyclic_Feedback(null);
    // Deserialize message field [feedback_id]
    data.feedback_id = GripperCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [status_flags]
    data.status_flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [fault_bank_a]
    data.fault_bank_a = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [fault_bank_b]
    data.fault_bank_b = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning_bank_a]
    data.warning_bank_a = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning_bank_b]
    data.warning_bank_b = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [motor]
    // Deserialize array length for message field [motor]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.motor = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motor[i] = MotorFeedback.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.motor.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GripperCyclic_Feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec3639b6684e8f718fd3128a8215e686';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    GripperCyclic_MessageId feedback_id
    uint32 status_flags
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    MotorFeedback[] motor
    ================================================================================
    MSG: kortex_driver/GripperCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/MotorFeedback
    
    uint32 motor_id
    float32 position
    float32 velocity
    float32 current_motor
    float32 voltage
    float32 temperature_motor
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperCyclic_Feedback(null);
    if (msg.feedback_id !== undefined) {
      resolved.feedback_id = GripperCyclic_MessageId.Resolve(msg.feedback_id)
    }
    else {
      resolved.feedback_id = new GripperCyclic_MessageId()
    }

    if (msg.status_flags !== undefined) {
      resolved.status_flags = msg.status_flags;
    }
    else {
      resolved.status_flags = 0
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

    if (msg.motor !== undefined) {
      resolved.motor = new Array(msg.motor.length);
      for (let i = 0; i < resolved.motor.length; ++i) {
        resolved.motor[i] = MotorFeedback.Resolve(msg.motor[i]);
      }
    }
    else {
      resolved.motor = []
    }

    return resolved;
    }
};

module.exports = GripperCyclic_Feedback;
