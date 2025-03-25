// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ConstrainedJointAngle = require('../msg/ConstrainedJointAngle.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class PlaySelectedJointTrajectoryRequest {
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
        this.input = new ConstrainedJointAngle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlaySelectedJointTrajectoryRequest
    // Serialize message field [input]
    bufferOffset = ConstrainedJointAngle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlaySelectedJointTrajectoryRequest
    let len;
    let data = new PlaySelectedJointTrajectoryRequest(null);
    // Deserialize message field [input]
    data.input = ConstrainedJointAngle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/PlaySelectedJointTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b5498a3a68bc52a621a3c2619c54c24';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ConstrainedJointAngle input
    
    ================================================================================
    MSG: kortex_driver/ConstrainedJointAngle
    
    uint32 joint_identifier
    float32 value
    JointTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/JointTrajectoryConstraint
    
    uint32 type
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlaySelectedJointTrajectoryRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ConstrainedJointAngle.Resolve(msg.input)
    }
    else {
      resolved.input = new ConstrainedJointAngle()
    }

    return resolved;
    }
};

class PlaySelectedJointTrajectoryResponse {
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
    // Serializes a message object of type PlaySelectedJointTrajectoryResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlaySelectedJointTrajectoryResponse
    let len;
    let data = new PlaySelectedJointTrajectoryResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/PlaySelectedJointTrajectoryResponse';
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
    const resolved = new PlaySelectedJointTrajectoryResponse(null);
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
  Request: PlaySelectedJointTrajectoryRequest,
  Response: PlaySelectedJointTrajectoryResponse,
  md5sum() { return '83e0097177e42b394a0d659ed427f041'; },
  datatype() { return 'kortex_driver/PlaySelectedJointTrajectory'; }
};
