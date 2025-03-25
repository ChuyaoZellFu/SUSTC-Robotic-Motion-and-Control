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

class BaseFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.active_state_connection_identifier = null;
      this.active_state = null;
      this.arm_voltage = null;
      this.arm_current = null;
      this.temperature_cpu = null;
      this.temperature_ambient = null;
      this.imu_acceleration_x = null;
      this.imu_acceleration_y = null;
      this.imu_acceleration_z = null;
      this.imu_angular_velocity_x = null;
      this.imu_angular_velocity_y = null;
      this.imu_angular_velocity_z = null;
      this.tool_pose_x = null;
      this.tool_pose_y = null;
      this.tool_pose_z = null;
      this.tool_pose_theta_x = null;
      this.tool_pose_theta_y = null;
      this.tool_pose_theta_z = null;
      this.tool_twist_linear_x = null;
      this.tool_twist_linear_y = null;
      this.tool_twist_linear_z = null;
      this.tool_twist_angular_x = null;
      this.tool_twist_angular_y = null;
      this.tool_twist_angular_z = null;
      this.tool_external_wrench_force_x = null;
      this.tool_external_wrench_force_y = null;
      this.tool_external_wrench_force_z = null;
      this.tool_external_wrench_torque_x = null;
      this.tool_external_wrench_torque_y = null;
      this.tool_external_wrench_torque_z = null;
      this.fault_bank_a = null;
      this.fault_bank_b = null;
      this.warning_bank_a = null;
      this.warning_bank_b = null;
      this.commanded_tool_pose_x = null;
      this.commanded_tool_pose_y = null;
      this.commanded_tool_pose_z = null;
      this.commanded_tool_pose_theta_x = null;
      this.commanded_tool_pose_theta_y = null;
      this.commanded_tool_pose_theta_z = null;
    }
    else {
      if (initObj.hasOwnProperty('active_state_connection_identifier')) {
        this.active_state_connection_identifier = initObj.active_state_connection_identifier
      }
      else {
        this.active_state_connection_identifier = 0;
      }
      if (initObj.hasOwnProperty('active_state')) {
        this.active_state = initObj.active_state
      }
      else {
        this.active_state = 0;
      }
      if (initObj.hasOwnProperty('arm_voltage')) {
        this.arm_voltage = initObj.arm_voltage
      }
      else {
        this.arm_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('arm_current')) {
        this.arm_current = initObj.arm_current
      }
      else {
        this.arm_current = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_cpu')) {
        this.temperature_cpu = initObj.temperature_cpu
      }
      else {
        this.temperature_cpu = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_ambient')) {
        this.temperature_ambient = initObj.temperature_ambient
      }
      else {
        this.temperature_ambient = 0.0;
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
      if (initObj.hasOwnProperty('tool_pose_x')) {
        this.tool_pose_x = initObj.tool_pose_x
      }
      else {
        this.tool_pose_x = 0.0;
      }
      if (initObj.hasOwnProperty('tool_pose_y')) {
        this.tool_pose_y = initObj.tool_pose_y
      }
      else {
        this.tool_pose_y = 0.0;
      }
      if (initObj.hasOwnProperty('tool_pose_z')) {
        this.tool_pose_z = initObj.tool_pose_z
      }
      else {
        this.tool_pose_z = 0.0;
      }
      if (initObj.hasOwnProperty('tool_pose_theta_x')) {
        this.tool_pose_theta_x = initObj.tool_pose_theta_x
      }
      else {
        this.tool_pose_theta_x = 0.0;
      }
      if (initObj.hasOwnProperty('tool_pose_theta_y')) {
        this.tool_pose_theta_y = initObj.tool_pose_theta_y
      }
      else {
        this.tool_pose_theta_y = 0.0;
      }
      if (initObj.hasOwnProperty('tool_pose_theta_z')) {
        this.tool_pose_theta_z = initObj.tool_pose_theta_z
      }
      else {
        this.tool_pose_theta_z = 0.0;
      }
      if (initObj.hasOwnProperty('tool_twist_linear_x')) {
        this.tool_twist_linear_x = initObj.tool_twist_linear_x
      }
      else {
        this.tool_twist_linear_x = 0.0;
      }
      if (initObj.hasOwnProperty('tool_twist_linear_y')) {
        this.tool_twist_linear_y = initObj.tool_twist_linear_y
      }
      else {
        this.tool_twist_linear_y = 0.0;
      }
      if (initObj.hasOwnProperty('tool_twist_linear_z')) {
        this.tool_twist_linear_z = initObj.tool_twist_linear_z
      }
      else {
        this.tool_twist_linear_z = 0.0;
      }
      if (initObj.hasOwnProperty('tool_twist_angular_x')) {
        this.tool_twist_angular_x = initObj.tool_twist_angular_x
      }
      else {
        this.tool_twist_angular_x = 0.0;
      }
      if (initObj.hasOwnProperty('tool_twist_angular_y')) {
        this.tool_twist_angular_y = initObj.tool_twist_angular_y
      }
      else {
        this.tool_twist_angular_y = 0.0;
      }
      if (initObj.hasOwnProperty('tool_twist_angular_z')) {
        this.tool_twist_angular_z = initObj.tool_twist_angular_z
      }
      else {
        this.tool_twist_angular_z = 0.0;
      }
      if (initObj.hasOwnProperty('tool_external_wrench_force_x')) {
        this.tool_external_wrench_force_x = initObj.tool_external_wrench_force_x
      }
      else {
        this.tool_external_wrench_force_x = 0.0;
      }
      if (initObj.hasOwnProperty('tool_external_wrench_force_y')) {
        this.tool_external_wrench_force_y = initObj.tool_external_wrench_force_y
      }
      else {
        this.tool_external_wrench_force_y = 0.0;
      }
      if (initObj.hasOwnProperty('tool_external_wrench_force_z')) {
        this.tool_external_wrench_force_z = initObj.tool_external_wrench_force_z
      }
      else {
        this.tool_external_wrench_force_z = 0.0;
      }
      if (initObj.hasOwnProperty('tool_external_wrench_torque_x')) {
        this.tool_external_wrench_torque_x = initObj.tool_external_wrench_torque_x
      }
      else {
        this.tool_external_wrench_torque_x = 0.0;
      }
      if (initObj.hasOwnProperty('tool_external_wrench_torque_y')) {
        this.tool_external_wrench_torque_y = initObj.tool_external_wrench_torque_y
      }
      else {
        this.tool_external_wrench_torque_y = 0.0;
      }
      if (initObj.hasOwnProperty('tool_external_wrench_torque_z')) {
        this.tool_external_wrench_torque_z = initObj.tool_external_wrench_torque_z
      }
      else {
        this.tool_external_wrench_torque_z = 0.0;
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
      if (initObj.hasOwnProperty('commanded_tool_pose_x')) {
        this.commanded_tool_pose_x = initObj.commanded_tool_pose_x
      }
      else {
        this.commanded_tool_pose_x = 0.0;
      }
      if (initObj.hasOwnProperty('commanded_tool_pose_y')) {
        this.commanded_tool_pose_y = initObj.commanded_tool_pose_y
      }
      else {
        this.commanded_tool_pose_y = 0.0;
      }
      if (initObj.hasOwnProperty('commanded_tool_pose_z')) {
        this.commanded_tool_pose_z = initObj.commanded_tool_pose_z
      }
      else {
        this.commanded_tool_pose_z = 0.0;
      }
      if (initObj.hasOwnProperty('commanded_tool_pose_theta_x')) {
        this.commanded_tool_pose_theta_x = initObj.commanded_tool_pose_theta_x
      }
      else {
        this.commanded_tool_pose_theta_x = 0.0;
      }
      if (initObj.hasOwnProperty('commanded_tool_pose_theta_y')) {
        this.commanded_tool_pose_theta_y = initObj.commanded_tool_pose_theta_y
      }
      else {
        this.commanded_tool_pose_theta_y = 0.0;
      }
      if (initObj.hasOwnProperty('commanded_tool_pose_theta_z')) {
        this.commanded_tool_pose_theta_z = initObj.commanded_tool_pose_theta_z
      }
      else {
        this.commanded_tool_pose_theta_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseFeedback
    // Serialize message field [active_state_connection_identifier]
    bufferOffset = _serializer.uint32(obj.active_state_connection_identifier, buffer, bufferOffset);
    // Serialize message field [active_state]
    bufferOffset = _serializer.uint32(obj.active_state, buffer, bufferOffset);
    // Serialize message field [arm_voltage]
    bufferOffset = _serializer.float32(obj.arm_voltage, buffer, bufferOffset);
    // Serialize message field [arm_current]
    bufferOffset = _serializer.float32(obj.arm_current, buffer, bufferOffset);
    // Serialize message field [temperature_cpu]
    bufferOffset = _serializer.float32(obj.temperature_cpu, buffer, bufferOffset);
    // Serialize message field [temperature_ambient]
    bufferOffset = _serializer.float32(obj.temperature_ambient, buffer, bufferOffset);
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
    // Serialize message field [tool_pose_x]
    bufferOffset = _serializer.float32(obj.tool_pose_x, buffer, bufferOffset);
    // Serialize message field [tool_pose_y]
    bufferOffset = _serializer.float32(obj.tool_pose_y, buffer, bufferOffset);
    // Serialize message field [tool_pose_z]
    bufferOffset = _serializer.float32(obj.tool_pose_z, buffer, bufferOffset);
    // Serialize message field [tool_pose_theta_x]
    bufferOffset = _serializer.float32(obj.tool_pose_theta_x, buffer, bufferOffset);
    // Serialize message field [tool_pose_theta_y]
    bufferOffset = _serializer.float32(obj.tool_pose_theta_y, buffer, bufferOffset);
    // Serialize message field [tool_pose_theta_z]
    bufferOffset = _serializer.float32(obj.tool_pose_theta_z, buffer, bufferOffset);
    // Serialize message field [tool_twist_linear_x]
    bufferOffset = _serializer.float32(obj.tool_twist_linear_x, buffer, bufferOffset);
    // Serialize message field [tool_twist_linear_y]
    bufferOffset = _serializer.float32(obj.tool_twist_linear_y, buffer, bufferOffset);
    // Serialize message field [tool_twist_linear_z]
    bufferOffset = _serializer.float32(obj.tool_twist_linear_z, buffer, bufferOffset);
    // Serialize message field [tool_twist_angular_x]
    bufferOffset = _serializer.float32(obj.tool_twist_angular_x, buffer, bufferOffset);
    // Serialize message field [tool_twist_angular_y]
    bufferOffset = _serializer.float32(obj.tool_twist_angular_y, buffer, bufferOffset);
    // Serialize message field [tool_twist_angular_z]
    bufferOffset = _serializer.float32(obj.tool_twist_angular_z, buffer, bufferOffset);
    // Serialize message field [tool_external_wrench_force_x]
    bufferOffset = _serializer.float32(obj.tool_external_wrench_force_x, buffer, bufferOffset);
    // Serialize message field [tool_external_wrench_force_y]
    bufferOffset = _serializer.float32(obj.tool_external_wrench_force_y, buffer, bufferOffset);
    // Serialize message field [tool_external_wrench_force_z]
    bufferOffset = _serializer.float32(obj.tool_external_wrench_force_z, buffer, bufferOffset);
    // Serialize message field [tool_external_wrench_torque_x]
    bufferOffset = _serializer.float32(obj.tool_external_wrench_torque_x, buffer, bufferOffset);
    // Serialize message field [tool_external_wrench_torque_y]
    bufferOffset = _serializer.float32(obj.tool_external_wrench_torque_y, buffer, bufferOffset);
    // Serialize message field [tool_external_wrench_torque_z]
    bufferOffset = _serializer.float32(obj.tool_external_wrench_torque_z, buffer, bufferOffset);
    // Serialize message field [fault_bank_a]
    bufferOffset = _serializer.uint32(obj.fault_bank_a, buffer, bufferOffset);
    // Serialize message field [fault_bank_b]
    bufferOffset = _serializer.uint32(obj.fault_bank_b, buffer, bufferOffset);
    // Serialize message field [warning_bank_a]
    bufferOffset = _serializer.uint32(obj.warning_bank_a, buffer, bufferOffset);
    // Serialize message field [warning_bank_b]
    bufferOffset = _serializer.uint32(obj.warning_bank_b, buffer, bufferOffset);
    // Serialize message field [commanded_tool_pose_x]
    bufferOffset = _serializer.float32(obj.commanded_tool_pose_x, buffer, bufferOffset);
    // Serialize message field [commanded_tool_pose_y]
    bufferOffset = _serializer.float32(obj.commanded_tool_pose_y, buffer, bufferOffset);
    // Serialize message field [commanded_tool_pose_z]
    bufferOffset = _serializer.float32(obj.commanded_tool_pose_z, buffer, bufferOffset);
    // Serialize message field [commanded_tool_pose_theta_x]
    bufferOffset = _serializer.float32(obj.commanded_tool_pose_theta_x, buffer, bufferOffset);
    // Serialize message field [commanded_tool_pose_theta_y]
    bufferOffset = _serializer.float32(obj.commanded_tool_pose_theta_y, buffer, bufferOffset);
    // Serialize message field [commanded_tool_pose_theta_z]
    bufferOffset = _serializer.float32(obj.commanded_tool_pose_theta_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseFeedback
    let len;
    let data = new BaseFeedback(null);
    // Deserialize message field [active_state_connection_identifier]
    data.active_state_connection_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [active_state]
    data.active_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [arm_voltage]
    data.arm_voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [arm_current]
    data.arm_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature_cpu]
    data.temperature_cpu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature_ambient]
    data.temperature_ambient = _deserializer.float32(buffer, bufferOffset);
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
    // Deserialize message field [tool_pose_x]
    data.tool_pose_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_pose_y]
    data.tool_pose_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_pose_z]
    data.tool_pose_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_pose_theta_x]
    data.tool_pose_theta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_pose_theta_y]
    data.tool_pose_theta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_pose_theta_z]
    data.tool_pose_theta_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_twist_linear_x]
    data.tool_twist_linear_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_twist_linear_y]
    data.tool_twist_linear_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_twist_linear_z]
    data.tool_twist_linear_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_twist_angular_x]
    data.tool_twist_angular_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_twist_angular_y]
    data.tool_twist_angular_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_twist_angular_z]
    data.tool_twist_angular_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_external_wrench_force_x]
    data.tool_external_wrench_force_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_external_wrench_force_y]
    data.tool_external_wrench_force_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_external_wrench_force_z]
    data.tool_external_wrench_force_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_external_wrench_torque_x]
    data.tool_external_wrench_torque_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_external_wrench_torque_y]
    data.tool_external_wrench_torque_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tool_external_wrench_torque_z]
    data.tool_external_wrench_torque_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fault_bank_a]
    data.fault_bank_a = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [fault_bank_b]
    data.fault_bank_b = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning_bank_a]
    data.warning_bank_a = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [warning_bank_b]
    data.warning_bank_b = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [commanded_tool_pose_x]
    data.commanded_tool_pose_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [commanded_tool_pose_y]
    data.commanded_tool_pose_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [commanded_tool_pose_z]
    data.commanded_tool_pose_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [commanded_tool_pose_theta_x]
    data.commanded_tool_pose_theta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [commanded_tool_pose_theta_y]
    data.commanded_tool_pose_theta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [commanded_tool_pose_theta_z]
    data.commanded_tool_pose_theta_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 160;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BaseFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '10860d64294ad841b25c45c5195aa64d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 active_state_connection_identifier
    uint32 active_state
    float32 arm_voltage
    float32 arm_current
    float32 temperature_cpu
    float32 temperature_ambient
    float32 imu_acceleration_x
    float32 imu_acceleration_y
    float32 imu_acceleration_z
    float32 imu_angular_velocity_x
    float32 imu_angular_velocity_y
    float32 imu_angular_velocity_z
    float32 tool_pose_x
    float32 tool_pose_y
    float32 tool_pose_z
    float32 tool_pose_theta_x
    float32 tool_pose_theta_y
    float32 tool_pose_theta_z
    float32 tool_twist_linear_x
    float32 tool_twist_linear_y
    float32 tool_twist_linear_z
    float32 tool_twist_angular_x
    float32 tool_twist_angular_y
    float32 tool_twist_angular_z
    float32 tool_external_wrench_force_x
    float32 tool_external_wrench_force_y
    float32 tool_external_wrench_force_z
    float32 tool_external_wrench_torque_x
    float32 tool_external_wrench_torque_y
    float32 tool_external_wrench_torque_z
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    float32 commanded_tool_pose_x
    float32 commanded_tool_pose_y
    float32 commanded_tool_pose_z
    float32 commanded_tool_pose_theta_x
    float32 commanded_tool_pose_theta_y
    float32 commanded_tool_pose_theta_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BaseFeedback(null);
    if (msg.active_state_connection_identifier !== undefined) {
      resolved.active_state_connection_identifier = msg.active_state_connection_identifier;
    }
    else {
      resolved.active_state_connection_identifier = 0
    }

    if (msg.active_state !== undefined) {
      resolved.active_state = msg.active_state;
    }
    else {
      resolved.active_state = 0
    }

    if (msg.arm_voltage !== undefined) {
      resolved.arm_voltage = msg.arm_voltage;
    }
    else {
      resolved.arm_voltage = 0.0
    }

    if (msg.arm_current !== undefined) {
      resolved.arm_current = msg.arm_current;
    }
    else {
      resolved.arm_current = 0.0
    }

    if (msg.temperature_cpu !== undefined) {
      resolved.temperature_cpu = msg.temperature_cpu;
    }
    else {
      resolved.temperature_cpu = 0.0
    }

    if (msg.temperature_ambient !== undefined) {
      resolved.temperature_ambient = msg.temperature_ambient;
    }
    else {
      resolved.temperature_ambient = 0.0
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

    if (msg.tool_pose_x !== undefined) {
      resolved.tool_pose_x = msg.tool_pose_x;
    }
    else {
      resolved.tool_pose_x = 0.0
    }

    if (msg.tool_pose_y !== undefined) {
      resolved.tool_pose_y = msg.tool_pose_y;
    }
    else {
      resolved.tool_pose_y = 0.0
    }

    if (msg.tool_pose_z !== undefined) {
      resolved.tool_pose_z = msg.tool_pose_z;
    }
    else {
      resolved.tool_pose_z = 0.0
    }

    if (msg.tool_pose_theta_x !== undefined) {
      resolved.tool_pose_theta_x = msg.tool_pose_theta_x;
    }
    else {
      resolved.tool_pose_theta_x = 0.0
    }

    if (msg.tool_pose_theta_y !== undefined) {
      resolved.tool_pose_theta_y = msg.tool_pose_theta_y;
    }
    else {
      resolved.tool_pose_theta_y = 0.0
    }

    if (msg.tool_pose_theta_z !== undefined) {
      resolved.tool_pose_theta_z = msg.tool_pose_theta_z;
    }
    else {
      resolved.tool_pose_theta_z = 0.0
    }

    if (msg.tool_twist_linear_x !== undefined) {
      resolved.tool_twist_linear_x = msg.tool_twist_linear_x;
    }
    else {
      resolved.tool_twist_linear_x = 0.0
    }

    if (msg.tool_twist_linear_y !== undefined) {
      resolved.tool_twist_linear_y = msg.tool_twist_linear_y;
    }
    else {
      resolved.tool_twist_linear_y = 0.0
    }

    if (msg.tool_twist_linear_z !== undefined) {
      resolved.tool_twist_linear_z = msg.tool_twist_linear_z;
    }
    else {
      resolved.tool_twist_linear_z = 0.0
    }

    if (msg.tool_twist_angular_x !== undefined) {
      resolved.tool_twist_angular_x = msg.tool_twist_angular_x;
    }
    else {
      resolved.tool_twist_angular_x = 0.0
    }

    if (msg.tool_twist_angular_y !== undefined) {
      resolved.tool_twist_angular_y = msg.tool_twist_angular_y;
    }
    else {
      resolved.tool_twist_angular_y = 0.0
    }

    if (msg.tool_twist_angular_z !== undefined) {
      resolved.tool_twist_angular_z = msg.tool_twist_angular_z;
    }
    else {
      resolved.tool_twist_angular_z = 0.0
    }

    if (msg.tool_external_wrench_force_x !== undefined) {
      resolved.tool_external_wrench_force_x = msg.tool_external_wrench_force_x;
    }
    else {
      resolved.tool_external_wrench_force_x = 0.0
    }

    if (msg.tool_external_wrench_force_y !== undefined) {
      resolved.tool_external_wrench_force_y = msg.tool_external_wrench_force_y;
    }
    else {
      resolved.tool_external_wrench_force_y = 0.0
    }

    if (msg.tool_external_wrench_force_z !== undefined) {
      resolved.tool_external_wrench_force_z = msg.tool_external_wrench_force_z;
    }
    else {
      resolved.tool_external_wrench_force_z = 0.0
    }

    if (msg.tool_external_wrench_torque_x !== undefined) {
      resolved.tool_external_wrench_torque_x = msg.tool_external_wrench_torque_x;
    }
    else {
      resolved.tool_external_wrench_torque_x = 0.0
    }

    if (msg.tool_external_wrench_torque_y !== undefined) {
      resolved.tool_external_wrench_torque_y = msg.tool_external_wrench_torque_y;
    }
    else {
      resolved.tool_external_wrench_torque_y = 0.0
    }

    if (msg.tool_external_wrench_torque_z !== undefined) {
      resolved.tool_external_wrench_torque_z = msg.tool_external_wrench_torque_z;
    }
    else {
      resolved.tool_external_wrench_torque_z = 0.0
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

    if (msg.commanded_tool_pose_x !== undefined) {
      resolved.commanded_tool_pose_x = msg.commanded_tool_pose_x;
    }
    else {
      resolved.commanded_tool_pose_x = 0.0
    }

    if (msg.commanded_tool_pose_y !== undefined) {
      resolved.commanded_tool_pose_y = msg.commanded_tool_pose_y;
    }
    else {
      resolved.commanded_tool_pose_y = 0.0
    }

    if (msg.commanded_tool_pose_z !== undefined) {
      resolved.commanded_tool_pose_z = msg.commanded_tool_pose_z;
    }
    else {
      resolved.commanded_tool_pose_z = 0.0
    }

    if (msg.commanded_tool_pose_theta_x !== undefined) {
      resolved.commanded_tool_pose_theta_x = msg.commanded_tool_pose_theta_x;
    }
    else {
      resolved.commanded_tool_pose_theta_x = 0.0
    }

    if (msg.commanded_tool_pose_theta_y !== undefined) {
      resolved.commanded_tool_pose_theta_y = msg.commanded_tool_pose_theta_y;
    }
    else {
      resolved.commanded_tool_pose_theta_y = 0.0
    }

    if (msg.commanded_tool_pose_theta_z !== undefined) {
      resolved.commanded_tool_pose_theta_z = msg.commanded_tool_pose_theta_z;
    }
    else {
      resolved.commanded_tool_pose_theta_z = 0.0
    }

    return resolved;
    }
};

module.exports = BaseFeedback;
