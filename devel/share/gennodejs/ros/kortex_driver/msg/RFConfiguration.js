// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WifiEnableState = require('./WifiEnableState.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');

//-----------------------------------------------------------

class RFConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wifi_enable_state = null;
      this.bluetooth_enable_state = null;
    }
    else {
      if (initObj.hasOwnProperty('wifi_enable_state')) {
        this.wifi_enable_state = initObj.wifi_enable_state
      }
      else {
        this.wifi_enable_state = new WifiEnableState();
      }
      if (initObj.hasOwnProperty('bluetooth_enable_state')) {
        this.bluetooth_enable_state = initObj.bluetooth_enable_state
      }
      else {
        this.bluetooth_enable_state = new BluetoothEnableState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RFConfiguration
    // Serialize message field [wifi_enable_state]
    bufferOffset = WifiEnableState.serialize(obj.wifi_enable_state, buffer, bufferOffset);
    // Serialize message field [bluetooth_enable_state]
    bufferOffset = BluetoothEnableState.serialize(obj.bluetooth_enable_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RFConfiguration
    let len;
    let data = new RFConfiguration(null);
    // Deserialize message field [wifi_enable_state]
    data.wifi_enable_state = WifiEnableState.deserialize(buffer, bufferOffset);
    // Deserialize message field [bluetooth_enable_state]
    data.bluetooth_enable_state = BluetoothEnableState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RFConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8bfe540c76a207913f7638bb2dbbae6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    WifiEnableState wifi_enable_state
    BluetoothEnableState bluetooth_enable_state
    ================================================================================
    MSG: kortex_driver/WifiEnableState
    
    bool enabled
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
    const resolved = new RFConfiguration(null);
    if (msg.wifi_enable_state !== undefined) {
      resolved.wifi_enable_state = WifiEnableState.Resolve(msg.wifi_enable_state)
    }
    else {
      resolved.wifi_enable_state = new WifiEnableState()
    }

    if (msg.bluetooth_enable_state !== undefined) {
      resolved.bluetooth_enable_state = BluetoothEnableState.Resolve(msg.bluetooth_enable_state)
    }
    else {
      resolved.bluetooth_enable_state = new BluetoothEnableState()
    }

    return resolved;
    }
};

module.exports = RFConfiguration;
