// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

let TrajectoryErrorReport = require('../msg/TrajectoryErrorReport.js');

//-----------------------------------------------------------

class GetTrajectoryErrorReportRequest {
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
        this.input = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTrajectoryErrorReportRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTrajectoryErrorReportRequest
    let len;
    let data = new GetTrajectoryErrorReportRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetTrajectoryErrorReportRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa3403cd5897c9698bc0fdcb2a453fbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty input
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTrajectoryErrorReportRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetTrajectoryErrorReportResponse {
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
        this.output = new TrajectoryErrorReport();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTrajectoryErrorReportResponse
    // Serialize message field [output]
    bufferOffset = TrajectoryErrorReport.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTrajectoryErrorReportResponse
    let len;
    let data = new GetTrajectoryErrorReportResponse(null);
    // Deserialize message field [output]
    data.output = TrajectoryErrorReport.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += TrajectoryErrorReport.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetTrajectoryErrorReportResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '78f80ad2dfce158708c08b1d353a0a4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TrajectoryErrorReport output
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTrajectoryErrorReportResponse(null);
    if (msg.output !== undefined) {
      resolved.output = TrajectoryErrorReport.Resolve(msg.output)
    }
    else {
      resolved.output = new TrajectoryErrorReport()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetTrajectoryErrorReportRequest,
  Response: GetTrajectoryErrorReportResponse,
  md5sum() { return '256dc258079fa086775c57ef6b093f4a'; },
  datatype() { return 'kortex_driver/GetTrajectoryErrorReport'; }
};
