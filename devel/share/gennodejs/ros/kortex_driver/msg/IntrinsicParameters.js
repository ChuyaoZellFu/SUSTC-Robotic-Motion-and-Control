// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DistortionCoefficients = require('./DistortionCoefficients.js');

//-----------------------------------------------------------

class IntrinsicParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
      this.resolution = null;
      this.principal_point_x = null;
      this.principal_point_y = null;
      this.focal_length_x = null;
      this.focal_length_y = null;
      this.distortion_coeffs = null;
    }
    else {
      if (initObj.hasOwnProperty('sensor')) {
        this.sensor = initObj.sensor
      }
      else {
        this.sensor = 0;
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0;
      }
      if (initObj.hasOwnProperty('principal_point_x')) {
        this.principal_point_x = initObj.principal_point_x
      }
      else {
        this.principal_point_x = 0.0;
      }
      if (initObj.hasOwnProperty('principal_point_y')) {
        this.principal_point_y = initObj.principal_point_y
      }
      else {
        this.principal_point_y = 0.0;
      }
      if (initObj.hasOwnProperty('focal_length_x')) {
        this.focal_length_x = initObj.focal_length_x
      }
      else {
        this.focal_length_x = 0.0;
      }
      if (initObj.hasOwnProperty('focal_length_y')) {
        this.focal_length_y = initObj.focal_length_y
      }
      else {
        this.focal_length_y = 0.0;
      }
      if (initObj.hasOwnProperty('distortion_coeffs')) {
        this.distortion_coeffs = initObj.distortion_coeffs
      }
      else {
        this.distortion_coeffs = new DistortionCoefficients();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IntrinsicParameters
    // Serialize message field [sensor]
    bufferOffset = _serializer.uint32(obj.sensor, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.uint32(obj.resolution, buffer, bufferOffset);
    // Serialize message field [principal_point_x]
    bufferOffset = _serializer.float32(obj.principal_point_x, buffer, bufferOffset);
    // Serialize message field [principal_point_y]
    bufferOffset = _serializer.float32(obj.principal_point_y, buffer, bufferOffset);
    // Serialize message field [focal_length_x]
    bufferOffset = _serializer.float32(obj.focal_length_x, buffer, bufferOffset);
    // Serialize message field [focal_length_y]
    bufferOffset = _serializer.float32(obj.focal_length_y, buffer, bufferOffset);
    // Serialize message field [distortion_coeffs]
    bufferOffset = DistortionCoefficients.serialize(obj.distortion_coeffs, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IntrinsicParameters
    let len;
    let data = new IntrinsicParameters(null);
    // Deserialize message field [sensor]
    data.sensor = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [principal_point_x]
    data.principal_point_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [principal_point_y]
    data.principal_point_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [focal_length_x]
    data.focal_length_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [focal_length_y]
    data.focal_length_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distortion_coeffs]
    data.distortion_coeffs = DistortionCoefficients.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/IntrinsicParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23b95f71707877a1e06e2ac8ba309bb9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sensor
    uint32 resolution
    float32 principal_point_x
    float32 principal_point_y
    float32 focal_length_x
    float32 focal_length_y
    DistortionCoefficients distortion_coeffs
    ================================================================================
    MSG: kortex_driver/DistortionCoefficients
    
    float32 k1
    float32 k2
    float32 k3
    float32 p1
    float32 p2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IntrinsicParameters(null);
    if (msg.sensor !== undefined) {
      resolved.sensor = msg.sensor;
    }
    else {
      resolved.sensor = 0
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0
    }

    if (msg.principal_point_x !== undefined) {
      resolved.principal_point_x = msg.principal_point_x;
    }
    else {
      resolved.principal_point_x = 0.0
    }

    if (msg.principal_point_y !== undefined) {
      resolved.principal_point_y = msg.principal_point_y;
    }
    else {
      resolved.principal_point_y = 0.0
    }

    if (msg.focal_length_x !== undefined) {
      resolved.focal_length_x = msg.focal_length_x;
    }
    else {
      resolved.focal_length_x = 0.0
    }

    if (msg.focal_length_y !== undefined) {
      resolved.focal_length_y = msg.focal_length_y;
    }
    else {
      resolved.focal_length_y = 0.0
    }

    if (msg.distortion_coeffs !== undefined) {
      resolved.distortion_coeffs = DistortionCoefficients.Resolve(msg.distortion_coeffs)
    }
    else {
      resolved.distortion_coeffs = new DistortionCoefficients()
    }

    return resolved;
    }
};

module.exports = IntrinsicParameters;
