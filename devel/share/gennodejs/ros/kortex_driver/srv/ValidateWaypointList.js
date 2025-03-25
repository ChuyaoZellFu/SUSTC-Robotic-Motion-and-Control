// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WaypointList = require('../msg/WaypointList.js');

//-----------------------------------------------------------

let WaypointValidationReport = require('../msg/WaypointValidationReport.js');

//-----------------------------------------------------------

class ValidateWaypointListRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new WaypointList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ValidateWaypointListRequest
    // Serialize message field [input]
    bufferOffset = WaypointList.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ValidateWaypointListRequest
    let len;
    let data = new ValidateWaypointListRequest(null);
    // Deserialize message field [input]
    data.input = WaypointList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WaypointList.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ValidateWaypointListRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9c355de078272f6acdb19f10ac9518d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WaypointList input
    
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
    const resolved = new ValidateWaypointListRequest(null);
    if (msg.input !== undefined) {
      resolved.input = WaypointList.Resolve(msg.input)
    }
    else {
      resolved.input = new WaypointList()
    }

    return resolved;
    }
};

class ValidateWaypointListResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = new WaypointValidationReport();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ValidateWaypointListResponse
    // Serialize message field [output]
    bufferOffset = WaypointValidationReport.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ValidateWaypointListResponse
    let len;
    let data = new ValidateWaypointListResponse(null);
    // Deserialize message field [output]
    data.output = WaypointValidationReport.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WaypointValidationReport.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ValidateWaypointListResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ab2f9c51d3b9ba949e1dea76a321fca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WaypointValidationReport output
    
    ================================================================================
    MSG: kortex_driver/WaypointValidationReport
    
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
    const resolved = new ValidateWaypointListResponse(null);
    if (msg.output !== undefined) {
      resolved.output = WaypointValidationReport.Resolve(msg.output)
    }
    else {
      resolved.output = new WaypointValidationReport()
    }

    return resolved;
    }
};

module.exports = {
  Request: ValidateWaypointListRequest,
  Response: ValidateWaypointListResponse,
  md5sum() { return '0b24f0cd37f005fabc6c65bffd727f77'; },
  datatype() { return 'kortex_driver/ValidateWaypointList'; }
};
