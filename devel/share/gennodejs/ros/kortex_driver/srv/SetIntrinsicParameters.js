// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let IntrinsicParameters = require('../msg/IntrinsicParameters.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetIntrinsicParametersRequest {
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
        this.input = new IntrinsicParameters();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIntrinsicParametersRequest
    // Serialize message field [input]
    bufferOffset = IntrinsicParameters.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIntrinsicParametersRequest
    let len;
    let data = new SetIntrinsicParametersRequest(null);
    // Deserialize message field [input]
    data.input = IntrinsicParameters.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetIntrinsicParametersRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c2a2b6647655afd9c96bec51c351692';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IntrinsicParameters input
    
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
    const resolved = new SetIntrinsicParametersRequest(null);
    if (msg.input !== undefined) {
      resolved.input = IntrinsicParameters.Resolve(msg.input)
    }
    else {
      resolved.input = new IntrinsicParameters()
    }

    return resolved;
    }
};

class SetIntrinsicParametersResponse {
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
    // Serializes a message object of type SetIntrinsicParametersResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIntrinsicParametersResponse
    let len;
    let data = new SetIntrinsicParametersResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetIntrinsicParametersResponse';
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
    const resolved = new SetIntrinsicParametersResponse(null);
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
  Request: SetIntrinsicParametersRequest,
  Response: SetIntrinsicParametersResponse,
  md5sum() { return 'e4fa2fa37ca4b5105af9a0b157505e8c'; },
  datatype() { return 'kortex_driver/SetIntrinsicParameters'; }
};
