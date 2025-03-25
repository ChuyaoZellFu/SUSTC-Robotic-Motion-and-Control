// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TwistCommand = require('./TwistCommand.js');
let WrenchCommand = require('./WrenchCommand.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let Snapshot = require('./Snapshot.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let ChangeTwist = require('./ChangeTwist.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let ChangeWrench = require('./ChangeWrench.js');
let EmergencyStop = require('./EmergencyStop.js');
let Faults = require('./Faults.js');
let Delay = require('./Delay.js');
let ActionHandle = require('./ActionHandle.js');
let GripperCommand = require('./GripperCommand.js');
let GpioCommand = require('./GpioCommand.js');
let Base_Stop = require('./Base_Stop.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let SequenceHandle = require('./SequenceHandle.js');
let WaypointList = require('./WaypointList.js');

//-----------------------------------------------------------

class Action_action_parameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.send_twist_command = null;
      this.send_wrench_command = null;
      this.send_joint_speeds = null;
      this.reach_pose = null;
      this.reach_joint_angles = null;
      this.toggle_admittance_mode = null;
      this.snapshot = null;
      this.switch_control_mapping = null;
      this.navigate_joints = null;
      this.navigate_mappings = null;
      this.change_twist = null;
      this.change_joint_speeds = null;
      this.change_wrench = null;
      this.apply_emergency_stop = null;
      this.clear_faults = null;
      this.delay = null;
      this.execute_action = null;
      this.send_gripper_command = null;
      this.send_gpio_command = null;
      this.stop_action = null;
      this.play_pre_computed_trajectory = null;
      this.execute_sequence = null;
      this.execute_waypoint_list = null;
    }
    else {
      if (initObj.hasOwnProperty('send_twist_command')) {
        this.send_twist_command = initObj.send_twist_command
      }
      else {
        this.send_twist_command = [];
      }
      if (initObj.hasOwnProperty('send_wrench_command')) {
        this.send_wrench_command = initObj.send_wrench_command
      }
      else {
        this.send_wrench_command = [];
      }
      if (initObj.hasOwnProperty('send_joint_speeds')) {
        this.send_joint_speeds = initObj.send_joint_speeds
      }
      else {
        this.send_joint_speeds = [];
      }
      if (initObj.hasOwnProperty('reach_pose')) {
        this.reach_pose = initObj.reach_pose
      }
      else {
        this.reach_pose = [];
      }
      if (initObj.hasOwnProperty('reach_joint_angles')) {
        this.reach_joint_angles = initObj.reach_joint_angles
      }
      else {
        this.reach_joint_angles = [];
      }
      if (initObj.hasOwnProperty('toggle_admittance_mode')) {
        this.toggle_admittance_mode = initObj.toggle_admittance_mode
      }
      else {
        this.toggle_admittance_mode = [];
      }
      if (initObj.hasOwnProperty('snapshot')) {
        this.snapshot = initObj.snapshot
      }
      else {
        this.snapshot = [];
      }
      if (initObj.hasOwnProperty('switch_control_mapping')) {
        this.switch_control_mapping = initObj.switch_control_mapping
      }
      else {
        this.switch_control_mapping = [];
      }
      if (initObj.hasOwnProperty('navigate_joints')) {
        this.navigate_joints = initObj.navigate_joints
      }
      else {
        this.navigate_joints = [];
      }
      if (initObj.hasOwnProperty('navigate_mappings')) {
        this.navigate_mappings = initObj.navigate_mappings
      }
      else {
        this.navigate_mappings = [];
      }
      if (initObj.hasOwnProperty('change_twist')) {
        this.change_twist = initObj.change_twist
      }
      else {
        this.change_twist = [];
      }
      if (initObj.hasOwnProperty('change_joint_speeds')) {
        this.change_joint_speeds = initObj.change_joint_speeds
      }
      else {
        this.change_joint_speeds = [];
      }
      if (initObj.hasOwnProperty('change_wrench')) {
        this.change_wrench = initObj.change_wrench
      }
      else {
        this.change_wrench = [];
      }
      if (initObj.hasOwnProperty('apply_emergency_stop')) {
        this.apply_emergency_stop = initObj.apply_emergency_stop
      }
      else {
        this.apply_emergency_stop = [];
      }
      if (initObj.hasOwnProperty('clear_faults')) {
        this.clear_faults = initObj.clear_faults
      }
      else {
        this.clear_faults = [];
      }
      if (initObj.hasOwnProperty('delay')) {
        this.delay = initObj.delay
      }
      else {
        this.delay = [];
      }
      if (initObj.hasOwnProperty('execute_action')) {
        this.execute_action = initObj.execute_action
      }
      else {
        this.execute_action = [];
      }
      if (initObj.hasOwnProperty('send_gripper_command')) {
        this.send_gripper_command = initObj.send_gripper_command
      }
      else {
        this.send_gripper_command = [];
      }
      if (initObj.hasOwnProperty('send_gpio_command')) {
        this.send_gpio_command = initObj.send_gpio_command
      }
      else {
        this.send_gpio_command = [];
      }
      if (initObj.hasOwnProperty('stop_action')) {
        this.stop_action = initObj.stop_action
      }
      else {
        this.stop_action = [];
      }
      if (initObj.hasOwnProperty('play_pre_computed_trajectory')) {
        this.play_pre_computed_trajectory = initObj.play_pre_computed_trajectory
      }
      else {
        this.play_pre_computed_trajectory = [];
      }
      if (initObj.hasOwnProperty('execute_sequence')) {
        this.execute_sequence = initObj.execute_sequence
      }
      else {
        this.execute_sequence = [];
      }
      if (initObj.hasOwnProperty('execute_waypoint_list')) {
        this.execute_waypoint_list = initObj.execute_waypoint_list
      }
      else {
        this.execute_waypoint_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Action_action_parameters
    // Serialize message field [send_twist_command]
    // Serialize the length for message field [send_twist_command]
    bufferOffset = _serializer.uint32(obj.send_twist_command.length, buffer, bufferOffset);
    obj.send_twist_command.forEach((val) => {
      bufferOffset = TwistCommand.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [send_wrench_command]
    // Serialize the length for message field [send_wrench_command]
    bufferOffset = _serializer.uint32(obj.send_wrench_command.length, buffer, bufferOffset);
    obj.send_wrench_command.forEach((val) => {
      bufferOffset = WrenchCommand.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [send_joint_speeds]
    // Serialize the length for message field [send_joint_speeds]
    bufferOffset = _serializer.uint32(obj.send_joint_speeds.length, buffer, bufferOffset);
    obj.send_joint_speeds.forEach((val) => {
      bufferOffset = Base_JointSpeeds.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [reach_pose]
    // Serialize the length for message field [reach_pose]
    bufferOffset = _serializer.uint32(obj.reach_pose.length, buffer, bufferOffset);
    obj.reach_pose.forEach((val) => {
      bufferOffset = ConstrainedPose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [reach_joint_angles]
    // Serialize the length for message field [reach_joint_angles]
    bufferOffset = _serializer.uint32(obj.reach_joint_angles.length, buffer, bufferOffset);
    obj.reach_joint_angles.forEach((val) => {
      bufferOffset = ConstrainedJointAngles.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [toggle_admittance_mode]
    bufferOffset = _arraySerializer.uint32(obj.toggle_admittance_mode, buffer, bufferOffset, null);
    // Serialize message field [snapshot]
    // Serialize the length for message field [snapshot]
    bufferOffset = _serializer.uint32(obj.snapshot.length, buffer, bufferOffset);
    obj.snapshot.forEach((val) => {
      bufferOffset = Snapshot.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [switch_control_mapping]
    // Serialize the length for message field [switch_control_mapping]
    bufferOffset = _serializer.uint32(obj.switch_control_mapping.length, buffer, bufferOffset);
    obj.switch_control_mapping.forEach((val) => {
      bufferOffset = SwitchControlMapping.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [navigate_joints]
    bufferOffset = _arraySerializer.uint32(obj.navigate_joints, buffer, bufferOffset, null);
    // Serialize message field [navigate_mappings]
    bufferOffset = _arraySerializer.uint32(obj.navigate_mappings, buffer, bufferOffset, null);
    // Serialize message field [change_twist]
    // Serialize the length for message field [change_twist]
    bufferOffset = _serializer.uint32(obj.change_twist.length, buffer, bufferOffset);
    obj.change_twist.forEach((val) => {
      bufferOffset = ChangeTwist.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [change_joint_speeds]
    // Serialize the length for message field [change_joint_speeds]
    bufferOffset = _serializer.uint32(obj.change_joint_speeds.length, buffer, bufferOffset);
    obj.change_joint_speeds.forEach((val) => {
      bufferOffset = ChangeJointSpeeds.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [change_wrench]
    // Serialize the length for message field [change_wrench]
    bufferOffset = _serializer.uint32(obj.change_wrench.length, buffer, bufferOffset);
    obj.change_wrench.forEach((val) => {
      bufferOffset = ChangeWrench.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [apply_emergency_stop]
    // Serialize the length for message field [apply_emergency_stop]
    bufferOffset = _serializer.uint32(obj.apply_emergency_stop.length, buffer, bufferOffset);
    obj.apply_emergency_stop.forEach((val) => {
      bufferOffset = EmergencyStop.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [clear_faults]
    // Serialize the length for message field [clear_faults]
    bufferOffset = _serializer.uint32(obj.clear_faults.length, buffer, bufferOffset);
    obj.clear_faults.forEach((val) => {
      bufferOffset = Faults.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [delay]
    // Serialize the length for message field [delay]
    bufferOffset = _serializer.uint32(obj.delay.length, buffer, bufferOffset);
    obj.delay.forEach((val) => {
      bufferOffset = Delay.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [execute_action]
    // Serialize the length for message field [execute_action]
    bufferOffset = _serializer.uint32(obj.execute_action.length, buffer, bufferOffset);
    obj.execute_action.forEach((val) => {
      bufferOffset = ActionHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [send_gripper_command]
    // Serialize the length for message field [send_gripper_command]
    bufferOffset = _serializer.uint32(obj.send_gripper_command.length, buffer, bufferOffset);
    obj.send_gripper_command.forEach((val) => {
      bufferOffset = GripperCommand.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [send_gpio_command]
    // Serialize the length for message field [send_gpio_command]
    bufferOffset = _serializer.uint32(obj.send_gpio_command.length, buffer, bufferOffset);
    obj.send_gpio_command.forEach((val) => {
      bufferOffset = GpioCommand.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [stop_action]
    // Serialize the length for message field [stop_action]
    bufferOffset = _serializer.uint32(obj.stop_action.length, buffer, bufferOffset);
    obj.stop_action.forEach((val) => {
      bufferOffset = Base_Stop.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [play_pre_computed_trajectory]
    // Serialize the length for message field [play_pre_computed_trajectory]
    bufferOffset = _serializer.uint32(obj.play_pre_computed_trajectory.length, buffer, bufferOffset);
    obj.play_pre_computed_trajectory.forEach((val) => {
      bufferOffset = PreComputedJointTrajectory.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [execute_sequence]
    // Serialize the length for message field [execute_sequence]
    bufferOffset = _serializer.uint32(obj.execute_sequence.length, buffer, bufferOffset);
    obj.execute_sequence.forEach((val) => {
      bufferOffset = SequenceHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [execute_waypoint_list]
    // Serialize the length for message field [execute_waypoint_list]
    bufferOffset = _serializer.uint32(obj.execute_waypoint_list.length, buffer, bufferOffset);
    obj.execute_waypoint_list.forEach((val) => {
      bufferOffset = WaypointList.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Action_action_parameters
    let len;
    let data = new Action_action_parameters(null);
    // Deserialize message field [send_twist_command]
    // Deserialize array length for message field [send_twist_command]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.send_twist_command = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.send_twist_command[i] = TwistCommand.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [send_wrench_command]
    // Deserialize array length for message field [send_wrench_command]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.send_wrench_command = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.send_wrench_command[i] = WrenchCommand.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [send_joint_speeds]
    // Deserialize array length for message field [send_joint_speeds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.send_joint_speeds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.send_joint_speeds[i] = Base_JointSpeeds.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [reach_pose]
    // Deserialize array length for message field [reach_pose]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.reach_pose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.reach_pose[i] = ConstrainedPose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [reach_joint_angles]
    // Deserialize array length for message field [reach_joint_angles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.reach_joint_angles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.reach_joint_angles[i] = ConstrainedJointAngles.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [toggle_admittance_mode]
    data.toggle_admittance_mode = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [snapshot]
    // Deserialize array length for message field [snapshot]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.snapshot = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.snapshot[i] = Snapshot.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [switch_control_mapping]
    // Deserialize array length for message field [switch_control_mapping]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.switch_control_mapping = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.switch_control_mapping[i] = SwitchControlMapping.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [navigate_joints]
    data.navigate_joints = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [navigate_mappings]
    data.navigate_mappings = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [change_twist]
    // Deserialize array length for message field [change_twist]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.change_twist = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.change_twist[i] = ChangeTwist.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [change_joint_speeds]
    // Deserialize array length for message field [change_joint_speeds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.change_joint_speeds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.change_joint_speeds[i] = ChangeJointSpeeds.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [change_wrench]
    // Deserialize array length for message field [change_wrench]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.change_wrench = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.change_wrench[i] = ChangeWrench.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [apply_emergency_stop]
    // Deserialize array length for message field [apply_emergency_stop]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.apply_emergency_stop = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.apply_emergency_stop[i] = EmergencyStop.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [clear_faults]
    // Deserialize array length for message field [clear_faults]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.clear_faults = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.clear_faults[i] = Faults.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [delay]
    // Deserialize array length for message field [delay]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.delay = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.delay[i] = Delay.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [execute_action]
    // Deserialize array length for message field [execute_action]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.execute_action = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.execute_action[i] = ActionHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [send_gripper_command]
    // Deserialize array length for message field [send_gripper_command]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.send_gripper_command = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.send_gripper_command[i] = GripperCommand.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [send_gpio_command]
    // Deserialize array length for message field [send_gpio_command]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.send_gpio_command = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.send_gpio_command[i] = GpioCommand.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [stop_action]
    // Deserialize array length for message field [stop_action]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.stop_action = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.stop_action[i] = Base_Stop.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [play_pre_computed_trajectory]
    // Deserialize array length for message field [play_pre_computed_trajectory]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.play_pre_computed_trajectory = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.play_pre_computed_trajectory[i] = PreComputedJointTrajectory.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [execute_sequence]
    // Deserialize array length for message field [execute_sequence]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.execute_sequence = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.execute_sequence[i] = SequenceHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [execute_waypoint_list]
    // Deserialize array length for message field [execute_waypoint_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.execute_waypoint_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.execute_waypoint_list[i] = WaypointList.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 32 * object.send_twist_command.length;
    length += 36 * object.send_wrench_command.length;
    object.send_joint_speeds.forEach((val) => {
      length += Base_JointSpeeds.getMessageSize(val);
    });
    object.reach_pose.forEach((val) => {
      length += ConstrainedPose.getMessageSize(val);
    });
    object.reach_joint_angles.forEach((val) => {
      length += ConstrainedJointAngles.getMessageSize(val);
    });
    length += 4 * object.toggle_admittance_mode.length;
    length += 4 * object.snapshot.length;
    length += 20 * object.switch_control_mapping.length;
    length += 4 * object.navigate_joints.length;
    length += 4 * object.navigate_mappings.length;
    length += 8 * object.change_twist.length;
    object.change_joint_speeds.forEach((val) => {
      length += ChangeJointSpeeds.getMessageSize(val);
    });
    length += 8 * object.change_wrench.length;
    length += 0 * object.apply_emergency_stop.length;
    length += 0 * object.clear_faults.length;
    length += 4 * object.delay.length;
    length += 12 * object.execute_action.length;
    object.send_gripper_command.forEach((val) => {
      length += GripperCommand.getMessageSize(val);
    });
    length += 16 * object.send_gpio_command.length;
    length += 0 * object.stop_action.length;
    object.play_pre_computed_trajectory.forEach((val) => {
      length += PreComputedJointTrajectory.getMessageSize(val);
    });
    length += 8 * object.execute_sequence.length;
    object.execute_waypoint_list.forEach((val) => {
      length += WaypointList.getMessageSize(val);
    });
    return length + 92;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Action_action_parameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d6115bf166bbc775637a0580d1e6e6c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    TwistCommand[] send_twist_command
    WrenchCommand[] send_wrench_command
    Base_JointSpeeds[] send_joint_speeds
    ConstrainedPose[] reach_pose
    ConstrainedJointAngles[] reach_joint_angles
    uint32[] toggle_admittance_mode
    Snapshot[] snapshot
    SwitchControlMapping[] switch_control_mapping
    uint32[] navigate_joints
    uint32[] navigate_mappings
    ChangeTwist[] change_twist
    ChangeJointSpeeds[] change_joint_speeds
    ChangeWrench[] change_wrench
    EmergencyStop[] apply_emergency_stop
    Faults[] clear_faults
    Delay[] delay
    ActionHandle[] execute_action
    GripperCommand[] send_gripper_command
    GpioCommand[] send_gpio_command
    Base_Stop[] stop_action
    PreComputedJointTrajectory[] play_pre_computed_trajectory
    SequenceHandle[] execute_sequence
    WaypointList[] execute_waypoint_list
    ================================================================================
    MSG: kortex_driver/TwistCommand
    
    uint32 reference_frame
    Twist twist
    uint32 duration
    ================================================================================
    MSG: kortex_driver/Twist
    
    float32 linear_x
    float32 linear_y
    float32 linear_z
    float32 angular_x
    float32 angular_y
    float32 angular_z
    ================================================================================
    MSG: kortex_driver/WrenchCommand
    
    uint32 reference_frame
    uint32 mode
    Wrench wrench
    uint32 duration
    ================================================================================
    MSG: kortex_driver/Wrench
    
    float32 force_x
    float32 force_y
    float32 force_z
    float32 torque_x
    float32 torque_y
    float32 torque_z
    ================================================================================
    MSG: kortex_driver/Base_JointSpeeds
    
    JointSpeed[] joint_speeds
    uint32 duration
    ================================================================================
    MSG: kortex_driver/JointSpeed
    
    uint32 joint_identifier
    float32 value
    uint32 duration
    ================================================================================
    MSG: kortex_driver/ConstrainedPose
    
    Pose target_pose
    CartesianTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/Pose
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    ================================================================================
    MSG: kortex_driver/CartesianTrajectoryConstraint
    
    CartesianTrajectoryConstraint_type oneof_type
    ================================================================================
    MSG: kortex_driver/CartesianTrajectoryConstraint_type
    
    CartesianSpeed[] speed
    uint32[] duration
    ================================================================================
    MSG: kortex_driver/CartesianSpeed
    
    float32 translation
    float32 orientation
    ================================================================================
    MSG: kortex_driver/ConstrainedJointAngles
    
    JointAngles joint_angles
    JointTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/JointAngles
    
    JointAngle[] joint_angles
    ================================================================================
    MSG: kortex_driver/JointAngle
    
    uint32 joint_identifier
    float32 value
    ================================================================================
    MSG: kortex_driver/JointTrajectoryConstraint
    
    uint32 type
    float32 value
    ================================================================================
    MSG: kortex_driver/Snapshot
    
    uint32 snapshot_type
    ================================================================================
    MSG: kortex_driver/SwitchControlMapping
    
    uint32 controller_identifier
    MapGroupHandle map_group_handle
    MapHandle map_handle
    ================================================================================
    MSG: kortex_driver/MapGroupHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/MapHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/ChangeTwist
    
    float32 linear
    float32 angular
    ================================================================================
    MSG: kortex_driver/ChangeJointSpeeds
    
    Base_JointSpeeds joint_speeds
    ================================================================================
    MSG: kortex_driver/ChangeWrench
    
    float32 force
    float32 torque
    ================================================================================
    MSG: kortex_driver/EmergencyStop
    
    ================================================================================
    MSG: kortex_driver/Faults
    
    ================================================================================
    MSG: kortex_driver/Delay
    
    uint32 duration
    ================================================================================
    MSG: kortex_driver/ActionHandle
    
    uint32 identifier
    uint32 action_type
    uint32 permission
    ================================================================================
    MSG: kortex_driver/GripperCommand
    
    uint32 mode
    Gripper gripper
    uint32 duration
    ================================================================================
    MSG: kortex_driver/Gripper
    
    Finger[] finger
    ================================================================================
    MSG: kortex_driver/Finger
    
    uint32 finger_identifier
    float32 value
    ================================================================================
    MSG: kortex_driver/GpioCommand
    
    uint32 port_identifier
    uint32 pin_identifier
    uint32 action
    uint32 period
    ================================================================================
    MSG: kortex_driver/Base_Stop
    
    ================================================================================
    MSG: kortex_driver/PreComputedJointTrajectory
    
    uint32 mode
    PreComputedJointTrajectoryElement[] trajectory_elements
    ================================================================================
    MSG: kortex_driver/PreComputedJointTrajectoryElement
    
    float32[] joint_angles
    float32[] joint_speeds
    float32[] joint_accelerations
    float32 time_from_start
    ================================================================================
    MSG: kortex_driver/SequenceHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/WaypointList
    
    Waypoint[] waypoints
    float32 duration
    bool use_optimal_blending
    ================================================================================
    MSG: kortex_driver/Waypoint
    
    string name
    Waypoint_type_of_waypoint oneof_type_of_waypoint
    ================================================================================
    MSG: kortex_driver/Waypoint_type_of_waypoint
    
    AngularWaypoint[] angular_waypoint
    CartesianWaypoint[] cartesian_waypoint
    ================================================================================
    MSG: kortex_driver/AngularWaypoint
    
    float32[] angles
    float32[] maximum_velocities
    float32 duration
    ================================================================================
    MSG: kortex_driver/CartesianWaypoint
    
    Pose pose
    uint32 reference_frame
    float32 maximum_linear_velocity
    float32 maximum_angular_velocity
    float32 blending_radius
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Action_action_parameters(null);
    if (msg.send_twist_command !== undefined) {
      resolved.send_twist_command = new Array(msg.send_twist_command.length);
      for (let i = 0; i < resolved.send_twist_command.length; ++i) {
        resolved.send_twist_command[i] = TwistCommand.Resolve(msg.send_twist_command[i]);
      }
    }
    else {
      resolved.send_twist_command = []
    }

    if (msg.send_wrench_command !== undefined) {
      resolved.send_wrench_command = new Array(msg.send_wrench_command.length);
      for (let i = 0; i < resolved.send_wrench_command.length; ++i) {
        resolved.send_wrench_command[i] = WrenchCommand.Resolve(msg.send_wrench_command[i]);
      }
    }
    else {
      resolved.send_wrench_command = []
    }

    if (msg.send_joint_speeds !== undefined) {
      resolved.send_joint_speeds = new Array(msg.send_joint_speeds.length);
      for (let i = 0; i < resolved.send_joint_speeds.length; ++i) {
        resolved.send_joint_speeds[i] = Base_JointSpeeds.Resolve(msg.send_joint_speeds[i]);
      }
    }
    else {
      resolved.send_joint_speeds = []
    }

    if (msg.reach_pose !== undefined) {
      resolved.reach_pose = new Array(msg.reach_pose.length);
      for (let i = 0; i < resolved.reach_pose.length; ++i) {
        resolved.reach_pose[i] = ConstrainedPose.Resolve(msg.reach_pose[i]);
      }
    }
    else {
      resolved.reach_pose = []
    }

    if (msg.reach_joint_angles !== undefined) {
      resolved.reach_joint_angles = new Array(msg.reach_joint_angles.length);
      for (let i = 0; i < resolved.reach_joint_angles.length; ++i) {
        resolved.reach_joint_angles[i] = ConstrainedJointAngles.Resolve(msg.reach_joint_angles[i]);
      }
    }
    else {
      resolved.reach_joint_angles = []
    }

    if (msg.toggle_admittance_mode !== undefined) {
      resolved.toggle_admittance_mode = msg.toggle_admittance_mode;
    }
    else {
      resolved.toggle_admittance_mode = []
    }

    if (msg.snapshot !== undefined) {
      resolved.snapshot = new Array(msg.snapshot.length);
      for (let i = 0; i < resolved.snapshot.length; ++i) {
        resolved.snapshot[i] = Snapshot.Resolve(msg.snapshot[i]);
      }
    }
    else {
      resolved.snapshot = []
    }

    if (msg.switch_control_mapping !== undefined) {
      resolved.switch_control_mapping = new Array(msg.switch_control_mapping.length);
      for (let i = 0; i < resolved.switch_control_mapping.length; ++i) {
        resolved.switch_control_mapping[i] = SwitchControlMapping.Resolve(msg.switch_control_mapping[i]);
      }
    }
    else {
      resolved.switch_control_mapping = []
    }

    if (msg.navigate_joints !== undefined) {
      resolved.navigate_joints = msg.navigate_joints;
    }
    else {
      resolved.navigate_joints = []
    }

    if (msg.navigate_mappings !== undefined) {
      resolved.navigate_mappings = msg.navigate_mappings;
    }
    else {
      resolved.navigate_mappings = []
    }

    if (msg.change_twist !== undefined) {
      resolved.change_twist = new Array(msg.change_twist.length);
      for (let i = 0; i < resolved.change_twist.length; ++i) {
        resolved.change_twist[i] = ChangeTwist.Resolve(msg.change_twist[i]);
      }
    }
    else {
      resolved.change_twist = []
    }

    if (msg.change_joint_speeds !== undefined) {
      resolved.change_joint_speeds = new Array(msg.change_joint_speeds.length);
      for (let i = 0; i < resolved.change_joint_speeds.length; ++i) {
        resolved.change_joint_speeds[i] = ChangeJointSpeeds.Resolve(msg.change_joint_speeds[i]);
      }
    }
    else {
      resolved.change_joint_speeds = []
    }

    if (msg.change_wrench !== undefined) {
      resolved.change_wrench = new Array(msg.change_wrench.length);
      for (let i = 0; i < resolved.change_wrench.length; ++i) {
        resolved.change_wrench[i] = ChangeWrench.Resolve(msg.change_wrench[i]);
      }
    }
    else {
      resolved.change_wrench = []
    }

    if (msg.apply_emergency_stop !== undefined) {
      resolved.apply_emergency_stop = new Array(msg.apply_emergency_stop.length);
      for (let i = 0; i < resolved.apply_emergency_stop.length; ++i) {
        resolved.apply_emergency_stop[i] = EmergencyStop.Resolve(msg.apply_emergency_stop[i]);
      }
    }
    else {
      resolved.apply_emergency_stop = []
    }

    if (msg.clear_faults !== undefined) {
      resolved.clear_faults = new Array(msg.clear_faults.length);
      for (let i = 0; i < resolved.clear_faults.length; ++i) {
        resolved.clear_faults[i] = Faults.Resolve(msg.clear_faults[i]);
      }
    }
    else {
      resolved.clear_faults = []
    }

    if (msg.delay !== undefined) {
      resolved.delay = new Array(msg.delay.length);
      for (let i = 0; i < resolved.delay.length; ++i) {
        resolved.delay[i] = Delay.Resolve(msg.delay[i]);
      }
    }
    else {
      resolved.delay = []
    }

    if (msg.execute_action !== undefined) {
      resolved.execute_action = new Array(msg.execute_action.length);
      for (let i = 0; i < resolved.execute_action.length; ++i) {
        resolved.execute_action[i] = ActionHandle.Resolve(msg.execute_action[i]);
      }
    }
    else {
      resolved.execute_action = []
    }

    if (msg.send_gripper_command !== undefined) {
      resolved.send_gripper_command = new Array(msg.send_gripper_command.length);
      for (let i = 0; i < resolved.send_gripper_command.length; ++i) {
        resolved.send_gripper_command[i] = GripperCommand.Resolve(msg.send_gripper_command[i]);
      }
    }
    else {
      resolved.send_gripper_command = []
    }

    if (msg.send_gpio_command !== undefined) {
      resolved.send_gpio_command = new Array(msg.send_gpio_command.length);
      for (let i = 0; i < resolved.send_gpio_command.length; ++i) {
        resolved.send_gpio_command[i] = GpioCommand.Resolve(msg.send_gpio_command[i]);
      }
    }
    else {
      resolved.send_gpio_command = []
    }

    if (msg.stop_action !== undefined) {
      resolved.stop_action = new Array(msg.stop_action.length);
      for (let i = 0; i < resolved.stop_action.length; ++i) {
        resolved.stop_action[i] = Base_Stop.Resolve(msg.stop_action[i]);
      }
    }
    else {
      resolved.stop_action = []
    }

    if (msg.play_pre_computed_trajectory !== undefined) {
      resolved.play_pre_computed_trajectory = new Array(msg.play_pre_computed_trajectory.length);
      for (let i = 0; i < resolved.play_pre_computed_trajectory.length; ++i) {
        resolved.play_pre_computed_trajectory[i] = PreComputedJointTrajectory.Resolve(msg.play_pre_computed_trajectory[i]);
      }
    }
    else {
      resolved.play_pre_computed_trajectory = []
    }

    if (msg.execute_sequence !== undefined) {
      resolved.execute_sequence = new Array(msg.execute_sequence.length);
      for (let i = 0; i < resolved.execute_sequence.length; ++i) {
        resolved.execute_sequence[i] = SequenceHandle.Resolve(msg.execute_sequence[i]);
      }
    }
    else {
      resolved.execute_sequence = []
    }

    if (msg.execute_waypoint_list !== undefined) {
      resolved.execute_waypoint_list = new Array(msg.execute_waypoint_list.length);
      for (let i = 0; i < resolved.execute_waypoint_list.length; ++i) {
        resolved.execute_waypoint_list[i] = WaypointList.Resolve(msg.execute_waypoint_list[i]);
      }
    }
    else {
      resolved.execute_waypoint_list = []
    }

    return resolved;
    }
};

module.exports = Action_action_parameters;
