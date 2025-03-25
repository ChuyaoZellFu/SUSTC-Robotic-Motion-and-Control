// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Calibration = require('../msg/Calibration.js');

//-----------------------------------------------------------

let CalibrationResult = require('../msg/CalibrationResult.js');

//-----------------------------------------------------------

class StopCalibrationRequest {
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
        this.input = new Calibration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StopCalibrationRequest
    // Serialize message field [input]
    bufferOffset = Calibration.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StopCalibrationRequest
    let len;
    let data = new StopCalibrationRequest(null);
    // Deserialize message field [input]
    data.input = Calibration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Calibration.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/StopCalibrationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68745c1f95256ccea9f0848f17f7fa0a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Calibration input
    
    ================================================================================
    MSG: kortex_driver/Calibration
    
    uint32 calibration_item
    CalibrationParameter[] calibration_parameter
    ================================================================================
    MSG: kortex_driver/CalibrationParameter
    
    uint32 calibration_parameter_identifier
    CalibrationParameter_value oneof_value
    ================================================================================
    MSG: kortex_driver/CalibrationParameter_value
    
    uint32[] signedIntValue
    uint32[] unsignedIntValue
    uint32[] floatValue
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StopCalibrationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Calibration.Resolve(msg.input)
    }
    else {
      resolved.input = new Calibration()
    }

    return resolved;
    }
};

class StopCalibrationResponse {
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
    // Serializes a message object of type StopCalibrationResponse
    // Serialize message field [output]
    bufferOffset = CalibrationResult.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StopCalibrationResponse
    let len;
    let data = new StopCalibrationResponse(null);
    // Deserialize message field [output]
    data.output = CalibrationResult.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/StopCalibrationResponse';
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
    const resolved = new StopCalibrationResponse(null);
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
  Request: StopCalibrationRequest,
  Response: StopCalibrationResponse,
  md5sum() { return 'd0dfa8caa770ffbbffd12276bb3d7f2c'; },
  datatype() { return 'kortex_driver/StopCalibration'; }
};
