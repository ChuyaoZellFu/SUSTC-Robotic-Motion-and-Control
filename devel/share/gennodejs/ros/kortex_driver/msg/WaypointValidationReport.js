// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let WaypointList = require('./WaypointList.js');

//-----------------------------------------------------------

class WaypointValidationReport {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory_error_report = null;
      this.optimal_waypoint_list = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory_error_report')) {
        this.trajectory_error_report = initObj.trajectory_error_report
      }
      else {
        this.trajectory_error_report = new TrajectoryErrorReport();
      }
      if (initObj.hasOwnProperty('optimal_waypoint_list')) {
        this.optimal_waypoint_list = initObj.optimal_waypoint_list
      }
      else {
        this.optimal_waypoint_list = new WaypointList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WaypointValidationReport
    // Serialize message field [trajectory_error_report]
    bufferOffset = TrajectoryErrorReport.serialize(obj.trajectory_error_report, buffer, bufferOffset);
    // Serialize message field [optimal_waypoint_list]
    bufferOffset = WaypointList.serialize(obj.optimal_waypoint_list, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WaypointValidationReport
    let len;
    let data = new WaypointValidationReport(null);
    // Deserialize message field [trajectory_error_report]
    data.trajectory_error_report = TrajectoryErrorReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [optimal_waypoint_list]
    data.optimal_waypoint_list = WaypointList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += TrajectoryErrorReport.getMessageSize(object.trajectory_error_report);
    length += WaypointList.getMessageSize(object.optimal_waypoint_list);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WaypointValidationReport';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7dd565ec93d4c9c5950e62db23f8114';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    TrajectoryErrorReport trajectory_error_report
    WaypointList optimal_waypoint_list
    ================================================================================
    MSG: kortex_driver/TrajectoryErrorReport
    
    TrajectoryErrorElement[] trajectory_error_elements
    ================================================================================
    MSG: kortex_driver/TrajectoryErrorElement
    
    uint32 error_type
    uint32 error_identifier
    float32 error_value
    float32 min_value
    float32 max_value
    uint32 index
    string message
    uint32 waypoint_index
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
    ================================================================================
    MSG: kortex_driver/Pose
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WaypointValidationReport(null);
    if (msg.trajectory_error_report !== undefined) {
      resolved.trajectory_error_report = TrajectoryErrorReport.Resolve(msg.trajectory_error_report)
    }
    else {
      resolved.trajectory_error_report = new TrajectoryErrorReport()
    }

    if (msg.optimal_waypoint_list !== undefined) {
      resolved.optimal_waypoint_list = WaypointList.Resolve(msg.optimal_waypoint_list)
    }
    else {
      resolved.optimal_waypoint_list = new WaypointList()
    }

    return resolved;
    }
};

module.exports = WaypointValidationReport;
