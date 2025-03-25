// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ExtrinsicParameters = require('../msg/ExtrinsicParameters.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetExtrinsicParametersRequest {
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
        this.input = new ExtrinsicParameters();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetExtrinsicParametersRequest
    // Serialize message field [input]
    bufferOffset = ExtrinsicParameters.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetExtrinsicParametersRequest
    let len;
    let data = new SetExtrinsicParametersRequest(null);
    // Deserialize message field [input]
    data.input = ExtrinsicParameters.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetExtrinsicParametersRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57323526fa45eb0ca8ea83e5c1d33654';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ExtrinsicParameters input
    
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
    const resolved = new SetExtrinsicParametersRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ExtrinsicParameters.Resolve(msg.input)
    }
    else {
      resolved.input = new ExtrinsicParameters()
    }

    return resolved;
    }
};

class SetExtrinsicParametersResponse {
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
    // Serializes a message object of type SetExtrinsicParametersResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetExtrinsicParametersResponse
    let len;
    let data = new SetExtrinsicParametersResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetExtrinsicParametersResponse';
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
    const resolved = new SetExtrinsicParametersResponse(null);
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
  Request: SetExtrinsicParametersRequest,
  Response: SetExtrinsicParametersResponse,
  md5sum() { return '4b2d31a314d526e9790309739cc1aeec'; },
  datatype() { return 'kortex_driver/SetExtrinsicParameters'; }
};
