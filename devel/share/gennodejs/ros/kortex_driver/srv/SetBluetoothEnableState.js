// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BluetoothEnableState = require('../msg/BluetoothEnableState.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetBluetoothEnableStateRequest {
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
        this.input = new BluetoothEnableState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBluetoothEnableStateRequest
    // Serialize message field [input]
    bufferOffset = BluetoothEnableState.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBluetoothEnableStateRequest
    let len;
    let data = new SetBluetoothEnableStateRequest(null);
    // Deserialize message field [input]
    data.input = BluetoothEnableState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetBluetoothEnableStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5db8254e8ad9ee9141adf349015e9ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BluetoothEnableState input
    
    ================================================================================
    MSG: kortex_driver/BluetoothEnableState
    
    bool enabled
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBluetoothEnableStateRequest(null);
    if (msg.input !== undefined) {
      resolved.input = BluetoothEnableState.Resolve(msg.input)
    }
    else {
      resolved.input = new BluetoothEnableState()
    }

    return resolved;
    }
};

class SetBluetoothEnableStateResponse {
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
    // Serializes a message object of type SetBluetoothEnableStateResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBluetoothEnableStateResponse
    let len;
    let data = new SetBluetoothEnableStateResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetBluetoothEnableStateResponse';
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
    const resolved = new SetBluetoothEnableStateResponse(null);
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
  Request: SetBluetoothEnableStateRequest,
  Response: SetBluetoothEnableStateResponse,
  md5sum() { return '7ccd6a6c56b1642433541444d0e2ab2f'; },
  datatype() { return 'kortex_driver/SetBluetoothEnableState'; }
};
