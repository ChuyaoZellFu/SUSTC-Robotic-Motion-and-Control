// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CalibrationElement = require('../msg/CalibrationElement.js');

//-----------------------------------------------------------

let CalibrationResult = require('../msg/CalibrationResult.js');

//-----------------------------------------------------------

class GetCalibrationResultRequest {
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
        this.input = new CalibrationElement();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCalibrationResultRequest
    // Serialize message field [input]
    bufferOffset = CalibrationElement.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCalibrationResultRequest
    let len;
    let data = new GetCalibrationResultRequest(null);
    // Deserialize message field [input]
    data.input = CalibrationElement.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetCalibrationResultRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45200d0cd663717cdf69c2c2b7047081';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CalibrationElement input
    
    ================================================================================
    MSG: kortex_driver/CalibrationElement
    
    uint32 calibration_item
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCalibrationResultRequest(null);
    if (msg.input !== undefined) {
      resolved.input = CalibrationElement.Resolve(msg.input)
    }
    else {
      resolved.input = new CalibrationElement()
    }

    return resolved;
    }
};

class GetCalibrationResultResponse {
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
        this.output = new CalibrationResult();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCalibrationResultResponse
    // Serialize message field [output]
    bufferOffset = CalibrationResult.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCalibrationResultResponse
    let len;
    let data = new GetCalibrationResultResponse(null);
    // Deserialize message field [output]
    data.output = CalibrationResult.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetCalibrationResultResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c30da30421956cb0c18ba7bbbeb697ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CalibrationResult output
    
    ================================================================================
    MSG: kortex_driver/CalibrationResult
    
    uint32 calibration_status
    uint32 calibration_details
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCalibrationResultResponse(null);
    if (msg.output !== undefined) {
      resolved.output = CalibrationResult.Resolve(msg.output)
    }
    else {
      resolved.output = new CalibrationResult()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCalibrationResultRequest,
  Response: GetCalibrationResultResponse,
  md5sum() { return '77b1f6ef7f1d5eace34aa81cf1772705'; },
  datatype() { return 'kortex_driver/GetCalibrationResult'; }
};
