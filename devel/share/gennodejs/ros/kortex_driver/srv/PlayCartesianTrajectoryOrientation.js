// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ConstrainedOrientation = require('../msg/ConstrainedOrientation.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class PlayCartesianTrajectoryOrientationRequest {
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
        this.input = new ConstrainedOrientation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlayCartesianTrajectoryOrientationRequest
    // Serialize message field [input]
    bufferOffset = ConstrainedOrientation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlayCartesianTrajectoryOrientationRequest
    let len;
    let data = new PlayCartesianTrajectoryOrientationRequest(null);
    // Deserialize message field [input]
    data.input = ConstrainedOrientation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ConstrainedOrientation.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/PlayCartesianTrajectoryOrientationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '96afc48e30e14106fe3a400f877b24b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ConstrainedOrientation input
    
    ================================================================================
    MSG: kortex_driver/ConstrainedOrientation
    
    Orientation target_orientation
    CartesianTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/Orientation
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlayCartesianTrajectoryOrientationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ConstrainedOrientation.Resolve(msg.input)
    }
    else {
      resolved.input = new ConstrainedOrientation()
    }

    return resolved;
    }
};

class PlayCartesianTrajectoryOrientationResponse {
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
        this.output = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlayCartesianTrajectoryOrientationResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlayCartesianTrajectoryOrientationResponse
    let len;
    let data = new PlayCartesianTrajectoryOrientationResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/PlayCartesianTrajectoryOrientationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6c43d221c810050f75091660f63b0cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty output
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlayCartesianTrajectoryOrientationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Empty.Resolve(msg.output)
    }
    else {
      resolved.output = new Empty()
    }

    return resolved;
    }
};

module.exports = {
  Request: PlayCartesianTrajectoryOrientationRequest,
  Response: PlayCartesianTrajectoryOrientationResponse,
  md5sum() { return '118c3171acc8dd69436f0f28b7925e31'; },
  datatype() { return 'kortex_driver/PlayCartesianTrajectoryOrientation'; }
};
