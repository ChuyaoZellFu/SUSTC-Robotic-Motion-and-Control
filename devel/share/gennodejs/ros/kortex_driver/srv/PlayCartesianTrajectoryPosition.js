// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ConstrainedPosition = require('../msg/ConstrainedPosition.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class PlayCartesianTrajectoryPositionRequest {
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
        this.input = new ConstrainedPosition();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlayCartesianTrajectoryPositionRequest
    // Serialize message field [input]
    bufferOffset = ConstrainedPosition.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlayCartesianTrajectoryPositionRequest
    let len;
    let data = new PlayCartesianTrajectoryPositionRequest(null);
    // Deserialize message field [input]
    data.input = ConstrainedPosition.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ConstrainedPosition.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/PlayCartesianTrajectoryPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e13526b094d2c2103287b183a9744f50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ConstrainedPosition input
    
    ================================================================================
    MSG: kortex_driver/ConstrainedPosition
    
    Base_Position target_position
    CartesianTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/Base_Position
    
    float32 x
    float32 y
    float32 z
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
    const resolved = new PlayCartesianTrajectoryPositionRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ConstrainedPosition.Resolve(msg.input)
    }
    else {
      resolved.input = new ConstrainedPosition()
    }

    return resolved;
    }
};

class PlayCartesianTrajectoryPositionResponse {
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
    // Serializes a message object of type PlayCartesianTrajectoryPositionResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlayCartesianTrajectoryPositionResponse
    let len;
    let data = new PlayCartesianTrajectoryPositionResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/PlayCartesianTrajectoryPositionResponse';
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
    const resolved = new PlayCartesianTrajectoryPositionResponse(null);
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
  Request: PlayCartesianTrajectoryPositionRequest,
  Response: PlayCartesianTrajectoryPositionResponse,
  md5sum() { return 'da1eaa51dfbdd2438263c790b42891de'; },
  datatype() { return 'kortex_driver/PlayCartesianTrajectoryPosition'; }
};
