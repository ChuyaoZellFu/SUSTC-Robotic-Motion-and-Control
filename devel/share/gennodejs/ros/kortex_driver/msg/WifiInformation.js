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

class WifiInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ssid = null;
      this.security_type = null;
      this.encryption_type = null;
      this.signal_quality = null;
      this.signal_strength = null;
      this.frequency = null;
      this.channel = null;
    }
    else {
      if (initObj.hasOwnProperty('ssid')) {
        this.ssid = initObj.ssid
      }
      else {
        this.ssid = new Ssid();
      }
      if (initObj.hasOwnProperty('security_type')) {
        this.security_type = initObj.security_type
      }
      else {
        this.security_type = 0;
      }
      if (initObj.hasOwnProperty('encryption_type')) {
        this.encryption_type = initObj.encryption_type
      }
      else {
        this.encryption_type = 0;
      }
      if (initObj.hasOwnProperty('signal_quality')) {
        this.signal_quality = initObj.signal_quality
      }
      else {
        this.signal_quality = 0;
      }
      if (initObj.hasOwnProperty('signal_strength')) {
        this.signal_strength = initObj.signal_strength
      }
      else {
        this.signal_strength = 0;
      }
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0;
      }
      if (initObj.hasOwnProperty('channel')) {
        this.channel = initObj.channel
      }
      else {
        this.channel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WifiInformation
    // Serialize message field [ssid]
    bufferOffset = Ssid.serialize(obj.ssid, buffer, bufferOffset);
    // Serialize message field [security_type]
    bufferOffset = _serializer.uint32(obj.security_type, buffer, bufferOffset);
    // Serialize message field [encryption_type]
    bufferOffset = _serializer.uint32(obj.encryption_type, buffer, bufferOffset);
    // Serialize message field [signal_quality]
    bufferOffset = _serializer.uint32(obj.signal_quality, buffer, bufferOffset);
    // Serialize message field [signal_strength]
    bufferOffset = _serializer.int32(obj.signal_strength, buffer, bufferOffset);
    // Serialize message field [frequency]
    bufferOffset = _serializer.uint32(obj.frequency, buffer, bufferOffset);
    // Serialize message field [channel]
    bufferOffset = _serializer.uint32(obj.channel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WifiInformation
    let len;
    let data = new WifiInformation(null);
    // Deserialize message field [ssid]
    data.ssid = Ssid.deserialize(buffer, bufferOffset);
    // Deserialize message field [security_type]
    data.security_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [encryption_type]
    data.encryption_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [signal_quality]
    data.signal_quality = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [signal_strength]
    data.signal_strength = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [channel]
    data.channel = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Ssid.getMessageSize(object.ssid);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WifiInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6cd167c73dec6e8468316540111c37c6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new WifiInformation(null);
    if (msg.ssid !== undefined) {
      resolved.ssid = Ssid.Resolve(msg.ssid)
    }
    else {
      resolved.ssid = new Ssid()
    }

    if (msg.security_type !== undefined) {
      resolved.security_type = msg.security_type;
    }
    else {
      resolved.security_type = 0
    }

    if (msg.encryption_type !== undefined) {
      resolved.encryption_type = msg.encryption_type;
    }
    else {
      resolved.encryption_type = 0
    }

    if (msg.signal_quality !== undefined) {
      resolved.signal_quality = msg.signal_quality;
    }
    else {
      resolved.signal_quality = 0
    }

    if (msg.signal_strength !== undefined) {
      resolved.signal_strength = msg.signal_strength;
    }
    else {
      resolved.signal_strength = 0
    }

    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0
    }

    if (msg.channel !== undefined) {
      resolved.channel = msg.channel;
    }
    else {
      resolved.channel = 0
    }

    return resolved;
    }
};

module.exports = WifiInformation;
