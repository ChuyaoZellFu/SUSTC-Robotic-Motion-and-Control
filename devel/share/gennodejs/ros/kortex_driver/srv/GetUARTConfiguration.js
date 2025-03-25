// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UARTDeviceIdentification = require('../msg/UARTDeviceIdentification.js');

//-----------------------------------------------------------

let UARTConfiguration = require('../msg/UARTConfiguration.js');

//-----------------------------------------------------------

class GetUARTConfigurationRequest {
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
        this.input = new UARTDeviceIdentification();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetUARTConfigurationRequest
    // Serialize message field [input]
    bufferOffset = UARTDeviceIdentification.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetUARTConfigurationRequest
    let len;
    let data = new GetUARTConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = UARTDeviceIdentification.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetUARTConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e34f297fb0dd637682ff6e1f3a7b31a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    UARTDeviceIdentification input
    
    ================================================================================
    MSG: kortex_driver/UARTDeviceIdentification
    
    uint32 port_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetUARTConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = UARTDeviceIdentification.Resolve(msg.input)
    }
    else {
      resolved.input = new UARTDeviceIdentification()
    }

    return resolved;
    }
};

class GetUARTConfigurationResponse {
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
        this.output = new UARTConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetUARTConfigurationResponse
    // Serialize message field [output]
    bufferOffset = UARTConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetUARTConfigurationResponse
    let len;
    let data = new GetUARTConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = UARTConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetUARTConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '691365f1855d76897edef52802563657';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    UARTConfiguration output
    
    ================================================================================
    MSG: kortex_driver/UARTConfiguration
    
    uint32 port_id
    bool enabled
    uint32 speed
    uint32 word_length
    uint32 stop_bits
    uint32 parity
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetUARTConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = UARTConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new UARTConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetUARTConfigurationRequest,
  Response: GetUARTConfigurationResponse,
  md5sum() { return 'd041631488869c35b5eb142c6744c8a7'; },
  datatype() { return 'kortex_driver/GetUARTConfiguration'; }
};
