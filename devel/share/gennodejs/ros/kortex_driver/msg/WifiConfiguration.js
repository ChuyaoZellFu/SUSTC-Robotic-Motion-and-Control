// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Ssid = require('./Ssid.js');

//-----------------------------------------------------------

class WifiConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ssid = null;
      this.security_key = null;
      this.connect_automatically = null;
    }
    else {
      if (initObj.hasOwnProperty('ssid')) {
        this.ssid = initObj.ssid
      }
      else {
        this.ssid = new Ssid();
      }
      if (initObj.hasOwnProperty('security_key')) {
        this.security_key = initObj.security_key
      }
      else {
        this.security_key = '';
      }
      if (initObj.hasOwnProperty('connect_automatically')) {
        this.connect_automatically = initObj.connect_automatically
      }
      else {
        this.connect_automatically = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WifiConfiguration
    // Serialize message field [ssid]
    bufferOffset = Ssid.serialize(obj.ssid, buffer, bufferOffset);
    // Serialize message field [security_key]
    bufferOffset = _serializer.string(obj.security_key, buffer, bufferOffset);
    // Serialize message field [connect_automatically]
    bufferOffset = _serializer.bool(obj.connect_automatically, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WifiConfiguration
    let len;
    let data = new WifiConfiguration(null);
    // Deserialize message field [ssid]
    data.ssid = Ssid.deserialize(buffer, bufferOffset);
    // Deserialize message field [security_key]
    data.security_key = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [connect_automatically]
    data.connect_automatically = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Ssid.getMessageSize(object.ssid);
    length += _getByteLength(object.security_key);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WifiConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4308400b333c62f7c2b092d1ea664706';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new WifiConfiguration(null);
    if (msg.ssid !== undefined) {
      resolved.ssid = Ssid.Resolve(msg.ssid)
    }
    else {
      resolved.ssid = new Ssid()
    }

    if (msg.security_key !== undefined) {
      resolved.security_key = msg.security_key;
    }
    else {
      resolved.security_key = ''
    }

    if (msg.connect_automatically !== undefined) {
      resolved.connect_automatically = msg.connect_automatically;
    }
    else {
      resolved.connect_automatically = false
    }

    return resolved;
    }
};

module.exports = WifiConfiguration;
