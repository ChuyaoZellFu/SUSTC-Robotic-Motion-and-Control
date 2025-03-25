// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ProductConfigurationEndEffectorType = require('../msg/ProductConfigurationEndEffectorType.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class UpdateEndEffectorTypeConfigurationRequest {
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
        this.input = new ProductConfigurationEndEffectorType();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UpdateEndEffectorTypeConfigurationRequest
    // Serialize message field [input]
    bufferOffset = ProductConfigurationEndEffectorType.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateEndEffectorTypeConfigurationRequest
    let len;
    let data = new UpdateEndEffectorTypeConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = ProductConfigurationEndEffectorType.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/UpdateEndEffectorTypeConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6db55183257243561f63029a7fc09ffe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ProductConfigurationEndEffectorType input
    
    ================================================================================
    MSG: kortex_driver/ProductConfigurationEndEffectorType
    
    uint32 end_effector_type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UpdateEndEffectorTypeConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ProductConfigurationEndEffectorType.Resolve(msg.input)
    }
    else {
      resolved.input = new ProductConfigurationEndEffectorType()
    }

    return resolved;
    }
};

class UpdateEndEffectorTypeConfigurationResponse {
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
    // Serializes a message object of type UpdateEndEffectorTypeConfigurationResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateEndEffectorTypeConfigurationResponse
    let len;
    let data = new UpdateEndEffectorTypeConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/UpdateEndEffectorTypeConfigurationResponse';
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
    const resolved = new UpdateEndEffectorTypeConfigurationResponse(null);
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
  Request: UpdateEndEffectorTypeConfigurationRequest,
  Response: UpdateEndEffectorTypeConfigurationResponse,
  md5sum() { return 'f022a6fb3cba8e0906c5a35f2f76f468'; },
  datatype() { return 'kortex_driver/UpdateEndEffectorTypeConfiguration'; }
};
