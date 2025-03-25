// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointAngles = require('../msg/JointAngles.js');

//-----------------------------------------------------------

let Pose = require('../msg/Pose.js');

//-----------------------------------------------------------

class ComputeForwardKinematicsRequest {
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
        this.input = new JointAngles();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeForwardKinematicsRequest
    // Serialize message field [input]
    bufferOffset = JointAngles.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeForwardKinematicsRequest
    let len;
    let data = new ComputeForwardKinematicsRequest(null);
    // Deserialize message field [input]
    data.input = JointAngles.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointAngles.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ComputeForwardKinematicsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44544f4fe6207865aaa76a373c777e04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointAngles input
    
    ================================================================================
    MSG: kortex_driver/JointAngles
    
    JointAngle[] joint_angles
    ================================================================================
    MSG: kortex_driver/JointAngle
    
    uint32 joint_identifier
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeForwardKinematicsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = JointAngles.Resolve(msg.input)
    }
    else {
      resolved.input = new JointAngles()
    }

    return resolved;
    }
};

class ComputeForwardKinematicsResponse {
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
        this.output = new Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeForwardKinematicsResponse
    // Serialize message field [output]
    bufferOffset = Pose.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeForwardKinematicsResponse
    let len;
    let data = new ComputeForwardKinematicsResponse(null);
    // Deserialize message field [output]
    data.output = Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ComputeForwardKinematicsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '688d02aea2316337d5932f2510ac9325';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Pose output
    
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
    const resolved = new ComputeForwardKinematicsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Pose.Resolve(msg.output)
    }
    else {
      resolved.output = new Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: ComputeForwardKinematicsRequest,
  Response: ComputeForwardKinematicsResponse,
  md5sum() { return '2406133d7d6bcd723f8f11457d1f2636'; },
  datatype() { return 'kortex_driver/ComputeForwardKinematics'; }
};
