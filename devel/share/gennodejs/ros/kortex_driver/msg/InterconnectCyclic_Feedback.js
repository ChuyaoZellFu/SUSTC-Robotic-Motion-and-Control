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
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');

//-----------------------------------------------------------

class InterconnectCyclic_Feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.feedback_id = null;
      this.status_flags = null;
      this.jitter_comm = null;
      this.imu_acceleration_x = null;
      this.imu_acceleration_y = null;
      this.imu_acceleration_z = null;
      this.imu_angular_velocity_x = null;
      this.imu_angular_velocity_y = null;
      this.imu_angular_velocity_z = null;
      this.voltage = null;
      this.temperature_core = null;
      this.fault_bank_a = null;
      this.fault_bank_b = null;
      this.warning_bank_a = null;
      this.warning_bank_b = null;
      this.oneof_tool_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('feedback_id')) {
        this.feedback_id = initObj.feedback_id
      }
      else {
        this.feedback_id = new InterconnectCyclic_MessageId();
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
      if (initObj.hasOwnProperty('imu_acceleration_x')) {
        this.imu_acceleration_x = initObj.imu_acceleration_x
      }
      else {
        this.imu_acceleration_x = 0.0;
      }
      if (initObj.hasOwnProperty('imu_acceleration_y')) {
        this.imu_acceleration_y = initObj.imu_acceleration_y
      }
      else {
        this.imu_acceleration_y = 0.0;
      }
      if (initObj.hasOwnProperty('imu_acceleration_z')) {
        this.imu_acceleration_z = initObj.imu_acceleration_z
      }
      else {
        this.imu_acceleration_z = 0.0;
      }
      if (initObj.hasOwnProperty('imu_angular_velocity_x')) {
        this.imu_angular_velocity_x = initObj.imu_angular_velocity_x
      }
      else {
        this.imu_angular_velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('imu_angular_velocity_y')) {
        this.imu_angular_velocity_y = initObj.imu_angular_velocity_y
      }
      else {
        this.imu_angular_velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('imu_angular_velocity_z')) {
        this.imu_angular_velocity_z = initObj.imu_angular_velocity_z
      }
      else {
        this.imu_angular_velocity_z = 0.0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
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
      if (initObj.hasOwnProperty('oneof_tool_feedback')) {
        this.oneof_tool_feedback = initObj.oneof_tool_feedback
      }
      else {
        this.oneof_tool_feedback = new InterconnectCyclic_Feedback_tool_feedback();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectCyclic_Feedback
    // Serialize message field [feedback_id]
    bufferOffset = InterconnectCyclic_MessageId.serialize(obj.feedback_id, buffer, bufferOffset);
    // Serialize message field [status_flags]
    bufferOffset = _serializer.uint32(obj.status_flags, buffer, bufferOffset);
    // Serialize message field [jitter_comm]
    bufferOffset = _serializer.uint32(obj.jitter_comm, buffer, bufferOffset);
    // Serialize message field [imu_acceleration_x]
    bufferOffset = _serializer.float32(obj.imu_acceleration_x, buffer, bufferOffset);
    // Serialize message field [imu_acceleration_y]
    bufferOffset = _serializer.float32(obj.imu_acceleration_y, buffer, bufferOffset);
    // Serialize message field [imu_acceleration_z]
    bufferOffset = _serializer.float32(obj.imu_acceleration_z, buffer, bufferOffset);
    // Serialize message field [imu_angular_velocity_x]
    bufferOffset = _serializer.float32(obj.imu_angular_velocity_x, buffer, bufferOffset);
    // Serialize message field [imu_angular_velocity_y]
    bufferOffset = _serializer.float32(obj.imu_angular_velocity_y, buffer, bufferOffset);
    // Serialize message field [imu_angular_velocity_z]
    bufferOffset = _serializer.float32(obj.imu_angular_velocity_z, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float32(obj.voltage, buffer, bufferOffset);
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
    // Serialize message field [oneof_tool_feedback]
    bufferOffset = InterconnectCyclic_Feedback_tool_feedback.serialize(obj.oneof_tool_feedback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectCyclic_Feedback
    let len;
    let data = new InterconnectCyclic_Feedback(null);
    // Deserialize message field [feedback_id]
    data.feedback_id = InterconnectCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [status_flags]
    data.status_flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [jitter_comm]
    data.jitter_comm = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [imu_acceleration_x]
    data.imu_acceleration_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_acceleration_y]
    data.imu_acceleration_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_acceleration_z]
    data.imu_acceleration_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_angular_velocity_x]
    data.imu_angular_velocity_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_angular_velocity_y]
    data.imu_angular_velocity_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_angular_velocity_z]
    data.imu_angular_velocity_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float32(buffer, bufferOffset);
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
    // Deserialize message field [oneof_tool_feedback]
    data.oneof_tool_feedback = InterconnectCyclic_Feedback_tool_feedback.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += InterconnectCyclic_Feedback_tool_feedback.getMessageSize(object.oneof_tool_feedback);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectCyclic_Feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '927bf07b9c0ece3890499e76c04f37e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    InterconnectCyclic_MessageId feedback_id
    uint32 status_flags
    uint32 jitter_comm
    float32 imu_acceleration_x
    float32 imu_acceleration_y
    float32 imu_acceleration_z
    float32 imu_angular_velocity_x
    float32 imu_angular_velocity_y
    float32 imu_angular_velocity_z
    float32 voltage
    float32 temperature_core
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback
    
    GripperCyclic_Feedback[] gripper_feedback
    ================================================================================
    MSG: kortex_driver/GripperCyclic_Feedback
    
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
    const resolved = new InterconnectCyclic_Feedback(null);
    if (msg.feedback_id !== undefined) {
      resolved.feedback_id = InterconnectCyclic_MessageId.Resolve(msg.feedback_id)
    }
    else {
      resolved.feedback_id = new InterconnectCyclic_MessageId()
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

    if (msg.imu_acceleration_x !== undefined) {
      resolved.imu_acceleration_x = msg.imu_acceleration_x;
    }
    else {
      resolved.imu_acceleration_x = 0.0
    }

    if (msg.imu_acceleration_y !== undefined) {
      resolved.imu_acceleration_y = msg.imu_acceleration_y;
    }
    else {
      resolved.imu_acceleration_y = 0.0
    }

    if (msg.imu_acceleration_z !== undefined) {
      resolved.imu_acceleration_z = msg.imu_acceleration_z;
    }
    else {
      resolved.imu_acceleration_z = 0.0
    }

    if (msg.imu_angular_velocity_x !== undefined) {
      resolved.imu_angular_velocity_x = msg.imu_angular_velocity_x;
    }
    else {
      resolved.imu_angular_velocity_x = 0.0
    }

    if (msg.imu_angular_velocity_y !== undefined) {
      resolved.imu_angular_velocity_y = msg.imu_angular_velocity_y;
    }
    else {
      resolved.imu_angular_velocity_y = 0.0
    }

    if (msg.imu_angular_velocity_z !== undefined) {
      resolved.imu_angular_velocity_z = msg.imu_angular_velocity_z;
    }
    else {
      resolved.imu_angular_velocity_z = 0.0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
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

    if (msg.oneof_tool_feedback !== undefined) {
      resolved.oneof_tool_feedback = InterconnectCyclic_Feedback_tool_feedback.Resolve(msg.oneof_tool_feedback)
    }
    else {
      resolved.oneof_tool_feedback = new InterconnectCyclic_Feedback_tool_feedback()
    }

    return resolved;
    }
};

module.exports = InterconnectCyclic_Feedback;
