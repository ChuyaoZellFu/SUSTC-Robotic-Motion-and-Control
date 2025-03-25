// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let IKData = require('../msg/IKData.js');

//-----------------------------------------------------------

let JointAngles = require('../msg/JointAngles.js');

//-----------------------------------------------------------

class ComputeInverseKinematicsRequest {
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
        this.input = new IKData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeInverseKinematicsRequest
    // Serialize message field [input]
    bufferOffset = IKData.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeInverseKinematicsRequest
    let len;
    let data = new ComputeInverseKinematicsRequest(null);
    // Deserialize message field [input]
    data.input = IKData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += IKData.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ComputeInverseKinematicsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6370dc05021b231d5d0b535bc12d00f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IKData input
    
    ================================================================================
    MSG: kortex_driver/IKData
    
    Pose cartesian_pose
    JointAngles guess
    ================================================================================
    MSG: kortex_driver/Pose
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
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
    const resolved = new ComputeInverseKinematicsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = IKData.Resolve(msg.input)
    }
    else {
      resolved.input = new IKData()
    }

    return resolved;
    }
};

class ComputeInverseKinematicsResponse {
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
        this.output = new JointAngles();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeInverseKinematicsResponse
    // Serialize message field [output]
    bufferOffset = JointAngles.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeInverseKinematicsResponse
    let len;
    let data = new ComputeInverseKinematicsResponse(null);
    // Deserialize message field [output]
    data.output = JointAngles.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointAngles.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ComputeInverseKinematicsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57ef940a22a8b164beebfd955a89b3bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointAngles output
    
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
    const resolved = new ComputeInverseKinematicsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = JointAngles.Resolve(msg.output)
    }
    else {
      resolved.output = new JointAngles()
    }

    return resolved;
    }
};

module.exports = {
  Request: ComputeInverseKinematicsRequest,
  Response: ComputeInverseKinematicsResponse,
  md5sum() { return '290825a1e5de199dc18075d261a5fee3'; },
  datatype() { return 'kortex_driver/ComputeInverseKinematics'; }
};
