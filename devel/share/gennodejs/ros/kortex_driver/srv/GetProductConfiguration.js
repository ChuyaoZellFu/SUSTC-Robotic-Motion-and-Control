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

let CompleteProductConfiguration = require('../msg/CompleteProductConfiguration.js');

//-----------------------------------------------------------

class GetProductConfigurationRequest {
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
    // Serializes a message object of type GetProductConfigurationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetProductConfigurationRequest
    let len;
    let data = new GetProductConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetProductConfigurationRequest';
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
    const resolved = new GetProductConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetProductConfigurationResponse {
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
        this.output = new CompleteProductConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetProductConfigurationResponse
    // Serialize message field [output]
    bufferOffset = CompleteProductConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetProductConfigurationResponse
    let len;
    let data = new GetProductConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = CompleteProductConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CompleteProductConfiguration.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetProductConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '98ed4d37d7247f3b94b20ec81a38583b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CompleteProductConfiguration output
    
    ================================================================================
    MSG: kortex_driver/CompleteProductConfiguration
    
    string kin
    uint32 model
    CountryCode country_code
    string assembly_plant
    string model_year
    uint32 degree_of_freedom
    uint32 base_type
    uint32 end_effector_type
    uint32 vision_module_type
    uint32 interface_module_type
    uint32 arm_laterality
    uint32 wrist_type
    uint32 brake_type
    ================================================================================
    MSG: kortex_driver/CountryCode
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetProductConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = CompleteProductConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new CompleteProductConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetProductConfigurationRequest,
  Response: GetProductConfigurationResponse,
  md5sum() { return '899fbdc53f9306591e652c7c52962785'; },
  datatype() { return 'kortex_driver/GetProductConfiguration'; }
};
