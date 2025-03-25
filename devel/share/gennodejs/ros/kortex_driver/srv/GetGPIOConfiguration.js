// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GPIOIdentification = require('../msg/GPIOIdentification.js');

//-----------------------------------------------------------

let InterconnectConfig_GPIOConfiguration = require('../msg/InterconnectConfig_GPIOConfiguration.js');

//-----------------------------------------------------------

class GetGPIOConfigurationRequest {
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
        this.input = new GPIOIdentification();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetGPIOConfigurationRequest
    // Serialize message field [input]
    bufferOffset = GPIOIdentification.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetGPIOConfigurationRequest
    let len;
    let data = new GetGPIOConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = GPIOIdentification.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetGPIOConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28fc5544fa28c41b8590a221a4f4db30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    GPIOIdentification input
    
    ================================================================================
    MSG: kortex_driver/GPIOIdentification
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetGPIOConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = GPIOIdentification.Resolve(msg.input)
    }
    else {
      resolved.input = new GPIOIdentification()
    }

    return resolved;
    }
};

class GetGPIOConfigurationResponse {
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
        this.output = new InterconnectConfig_GPIOConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetGPIOConfigurationResponse
    // Serialize message field [output]
    bufferOffset = InterconnectConfig_GPIOConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetGPIOConfigurationResponse
    let len;
    let data = new GetGPIOConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = InterconnectConfig_GPIOConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetGPIOConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '112c99a6fcf092b1686b2f73729789a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    InterconnectConfig_GPIOConfiguration output
    
    ================================================================================
    MSG: kortex_driver/InterconnectConfig_GPIOConfiguration
    
    uint32 identifier
    uint32 mode
    uint32 pull
    uint32 default_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetGPIOConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = InterconnectConfig_GPIOConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new InterconnectConfig_GPIOConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetGPIOConfigurationRequest,
  Response: GetGPIOConfigurationResponse,
  md5sum() { return '54322e7ee9a9cb34b1b8f34c41c5ae4d'; },
  datatype() { return 'kortex_driver/GetGPIOConfiguration'; }
};
