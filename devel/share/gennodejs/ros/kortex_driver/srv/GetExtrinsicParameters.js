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

let ExtrinsicParameters = require('../msg/ExtrinsicParameters.js');

//-----------------------------------------------------------

class GetExtrinsicParametersRequest {
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
    // Serializes a message object of type GetExtrinsicParametersRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetExtrinsicParametersRequest
    let len;
    let data = new GetExtrinsicParametersRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetExtrinsicParametersRequest';
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
    const resolved = new GetExtrinsicParametersRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetExtrinsicParametersResponse {
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
        this.output = new ExtrinsicParameters();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetExtrinsicParametersResponse
    // Serialize message field [output]
    bufferOffset = ExtrinsicParameters.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetExtrinsicParametersResponse
    let len;
    let data = new GetExtrinsicParametersResponse(null);
    // Deserialize message field [output]
    data.output = ExtrinsicParameters.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetExtrinsicParametersResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb1b8c70a178c89f172f8b8ef299947b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ExtrinsicParameters output
    
    ================================================================================
    MSG: kortex_driver/ExtrinsicParameters
    
    VisionConfig_RotationMatrix rotation
    TranslationVector translation
    ================================================================================
    MSG: kortex_driver/VisionConfig_RotationMatrix
    
    VisionConfig_RotationMatrixRow row1
    VisionConfig_RotationMatrixRow row2
    VisionConfig_RotationMatrixRow row3
    ================================================================================
    MSG: kortex_driver/VisionConfig_RotationMatrixRow
    
    float32 column1
    float32 column2
    float32 column3
    ================================================================================
    MSG: kortex_driver/TranslationVector
    
    float32 t_x
    float32 t_y
    float32 t_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetExtrinsicParametersResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ExtrinsicParameters.Resolve(msg.output)
    }
    else {
      resolved.output = new ExtrinsicParameters()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetExtrinsicParametersRequest,
  Response: GetExtrinsicParametersResponse,
  md5sum() { return '769b6e0a4e2cbed0ce69ce84dc1b50e3'; },
  datatype() { return 'kortex_driver/GetExtrinsicParameters'; }
};
