// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');

//-----------------------------------------------------------

class TrajectoryErrorReport {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory_error_elements = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory_error_elements')) {
        this.trajectory_error_elements = initObj.trajectory_error_elements
      }
      else {
        this.trajectory_error_elements = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryErrorReport
    // Serialize message field [trajectory_error_elements]
    // Serialize the length for message field [trajectory_error_elements]
    bufferOffset = _serializer.uint32(obj.trajectory_error_elements.length, buffer, bufferOffset);
    obj.trajectory_error_elements.forEach((val) => {
      bufferOffset = TrajectoryErrorElement.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryErrorReport
    let len;
    let data = new TrajectoryErrorReport(null);
    // Deserialize message field [trajectory_error_elements]
    // Deserialize array length for message field [trajectory_error_elements]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.trajectory_error_elements = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.trajectory_error_elements[i] = TrajectoryErrorElement.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.trajectory_error_elements.forEach((val) => {
      length += TrajectoryErrorElement.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TrajectoryErrorReport';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a94c619ffdcf2a4bccd610a99b960b54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new TrajectoryErrorReport(null);
    if (msg.trajectory_error_elements !== undefined) {
      resolved.trajectory_error_elements = new Array(msg.trajectory_error_elements.length);
      for (let i = 0; i < resolved.trajectory_error_elements.length; ++i) {
        resolved.trajectory_error_elements[i] = TrajectoryErrorElement.Resolve(msg.trajectory_error_elements[i]);
      }
    }
    else {
      resolved.trajectory_error_elements = []
    }

    return resolved;
    }
};

module.exports = TrajectoryErrorReport;
