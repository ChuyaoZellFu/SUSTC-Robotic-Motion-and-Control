// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SensorIdentifier = require('../msg/SensorIdentifier.js');

//-----------------------------------------------------------

let IntrinsicParameters = require('../msg/IntrinsicParameters.js');

//-----------------------------------------------------------

class GetIntrinsicParametersRequest {
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
        this.input = new SensorIdentifier();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIntrinsicParametersRequest
    // Serialize message field [input]
    bufferOffset = SensorIdentifier.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIntrinsicParametersRequest
    let len;
    let data = new GetIntrinsicParametersRequest(null);
    // Deserialize message field [input]
    data.input = SensorIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIntrinsicParametersRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6b665ee68fb76fe11bbb8a3f0942a78';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SensorIdentifier input
    
    ================================================================================
    MSG: kortex_driver/SensorIdentifier
    
    uint32 sensor
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetIntrinsicParametersRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SensorIdentifier.Resolve(msg.input)
    }
    else {
      resolved.input = new SensorIdentifier()
    }

    return resolved;
    }
};

class GetIntrinsicParametersResponse {
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
        this.output = new IntrinsicParameters();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIntrinsicParametersResponse
    // Serialize message field [output]
    bufferOffset = IntrinsicParameters.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIntrinsicParametersResponse
    let len;
    let data = new GetIntrinsicParametersResponse(null);
    // Deserialize message field [output]
    data.output = IntrinsicParameters.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIntrinsicParametersResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '798e895c21bee34b09bca25c4e3ffe07';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IntrinsicParameters output
    
    ================================================================================
    MSG: kortex_driver/IntrinsicParameters
    
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
    const resolved = new GetIntrinsicParametersResponse(null);
    if (msg.output !== undefined) {
      resolved.output = IntrinsicParameters.Resolve(msg.output)
    }
    else {
      resolved.output = new IntrinsicParameters()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetIntrinsicParametersRequest,
  Response: GetIntrinsicParametersResponse,
  md5sum() { return 'b088fe2711ccce33e17ff2a6a6244b06'; },
  datatype() { return 'kortex_driver/GetIntrinsicParameters'; }
};
