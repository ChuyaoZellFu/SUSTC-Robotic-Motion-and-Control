// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WifiInformation = require('./WifiInformation.js');

//-----------------------------------------------------------

class WifiInformationList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wifi_information_list = null;
    }
    else {
      if (initObj.hasOwnProperty('wifi_information_list')) {
        this.wifi_information_list = initObj.wifi_information_list
      }
      else {
        this.wifi_information_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WifiInformationList
    // Serialize message field [wifi_information_list]
    // Serialize the length for message field [wifi_information_list]
    bufferOffset = _serializer.uint32(obj.wifi_information_list.length, buffer, bufferOffset);
    obj.wifi_information_list.forEach((val) => {
      bufferOffset = WifiInformation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WifiInformationList
    let len;
    let data = new WifiInformationList(null);
    // Deserialize message field [wifi_information_list]
    // Deserialize array length for message field [wifi_information_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.wifi_information_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.wifi_information_list[i] = WifiInformation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.wifi_information_list.forEach((val) => {
      length += WifiInformation.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WifiInformationList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e35665ac97a2d2d722059d5f54d85ef4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    WifiInformation[] wifi_information_list
    ================================================================================
    MSG: kortex_driver/WifiInformation
    
    Ssid ssid
    uint32 security_type
    uint32 encryption_type
    uint32 signal_quality
    int32 signal_strength
    uint32 frequency
    uint32 channel
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
    const resolved = new WifiInformationList(null);
    if (msg.wifi_information_list !== undefined) {
      resolved.wifi_information_list = new Array(msg.wifi_information_list.length);
      for (let i = 0; i < resolved.wifi_information_list.length; ++i) {
        resolved.wifi_information_list[i] = WifiInformation.Resolve(msg.wifi_information_list[i]);
      }
    }
    else {
      resolved.wifi_information_list = []
    }

    return resolved;
    }
};

module.exports = WifiInformationList;
