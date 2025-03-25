// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WifiConfiguration = require('./WifiConfiguration.js');

//-----------------------------------------------------------

class WifiConfigurationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wifi_configuration_list = null;
    }
    else {
      if (initObj.hasOwnProperty('wifi_configuration_list')) {
        this.wifi_configuration_list = initObj.wifi_configuration_list
      }
      else {
        this.wifi_configuration_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WifiConfigurationList
    // Serialize message field [wifi_configuration_list]
    // Serialize the length for message field [wifi_configuration_list]
    bufferOffset = _serializer.uint32(obj.wifi_configuration_list.length, buffer, bufferOffset);
    obj.wifi_configuration_list.forEach((val) => {
      bufferOffset = WifiConfiguration.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WifiConfigurationList
    let len;
    let data = new WifiConfigurationList(null);
    // Deserialize message field [wifi_configuration_list]
    // Deserialize array length for message field [wifi_configuration_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.wifi_configuration_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.wifi_configuration_list[i] = WifiConfiguration.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.wifi_configuration_list.forEach((val) => {
      length += WifiConfiguration.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WifiConfigurationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '826e37c04fad51b5554819fbd5e69cc8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    WifiConfiguration[] wifi_configuration_list
    ================================================================================
    MSG: kortex_driver/WifiConfiguration
    
    Ssid ssid
    string security_key
    bool connect_automatically
    ================================================================================
    MSG: kortex_driver/Ssid
    
    string identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WifiConfigurationList(null);
    if (msg.wifi_configuration_list !== undefined) {
      resolved.wifi_configuration_list = new Array(msg.wifi_configuration_list.length);
      for (let i = 0; i < resolved.wifi_configuration_list.length; ++i) {
        resolved.wifi_configuration_list[i] = WifiConfiguration.Resolve(msg.wifi_configuration_list[i]);
      }
    }
    else {
      resolved.wifi_configuration_list = []
    }

    return resolved;
    }
};

module.exports = WifiConfigurationList;
