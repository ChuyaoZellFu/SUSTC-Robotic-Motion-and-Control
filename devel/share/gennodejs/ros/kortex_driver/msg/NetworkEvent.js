// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class NetworkEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NetworkEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NetworkEvent
    let len;
    let data = new NetworkEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/NetworkEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '342adcc6d6071ab5d5d0b6d3b3ff3e84';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_NETWORK_EVENT = 0
    
    uint32 WIFI_CONNECTED = 1
    
    uint32 WIFI_DISCONNECTED = 2
    
    uint32 WIFI_SCAN_STARTED = 3
    
    uint32 WIFI_SCAN_RESULTS = 4
    
    uint32 WIFI_SCAN_FAILED = 5
    
    uint32 WIFI_NOT_FOUND = 6
    
    uint32 WIFI_ASSOC_REJECTED = 7
    
    uint32 WIFI_AUTH_WRONG_KEY = 8
    
    uint32 WIFI_AUTH_CONN_FAILED = 9
    
    uint32 WIFI_AUTH_FAILED = 10
    
    uint32 WIFI_ENABLED = 11
    
    uint32 WIFI_DISABLED = 12
    
    uint32 BLUETOOTH_ENABLED = 13
    
    uint32 BLUETOOTH_DISABLED = 14
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NetworkEvent(null);
    return resolved;
    }
};

// Constants for message
NetworkEvent.Constants = {
  UNSPECIFIED_NETWORK_EVENT: 0,
  WIFI_CONNECTED: 1,
  WIFI_DISCONNECTED: 2,
  WIFI_SCAN_STARTED: 3,
  WIFI_SCAN_RESULTS: 4,
  WIFI_SCAN_FAILED: 5,
  WIFI_NOT_FOUND: 6,
  WIFI_ASSOC_REJECTED: 7,
  WIFI_AUTH_WRONG_KEY: 8,
  WIFI_AUTH_CONN_FAILED: 9,
  WIFI_AUTH_FAILED: 10,
  WIFI_ENABLED: 11,
  WIFI_DISABLED: 12,
  BLUETOOTH_ENABLED: 13,
  BLUETOOTH_DISABLED: 14,
}

module.exports = NetworkEvent;
